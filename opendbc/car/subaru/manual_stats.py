"""
Manual Transmission Driving Stats Tracker

Tracks and analyzes manual driving behavior:
- Stalls: Engine dies while trying to move
- Lugging: Driving at too-low RPM under load
- Shift quality: How well upshifts and downshifts are executed
- Launch smoothness: How smoothly the car starts from a stop
- Gear prediction from RPM and speed
- Shift suggestions for economy/performance

Called from Subaru carstate.py on each update.
"""

import json
import time
import math
from dataclasses import dataclass, field, asdict
from typing import Optional
from collections import deque

from openpilot.common.params import Params
from opendbc.car import DT_CTRL


# 2024 Subaru BRZ 6MT gear ratios
BRZ_GEAR_RATIOS = {
  1: 3.626,
  2: 2.188,
  3: 1.541,
  4: 1.213,
  5: 1.000,
  6: 0.767,
}
BRZ_FINAL_DRIVE = 4.10
# 2024 BRZ Limited: 215/40R18 tires on 18" wheels
# Diameter = 18" (457.2mm) + 2 * (215mm * 0.40) = 629.2mm
# Circumference = π * 0.6292m = 1.977m
BRZ_TIRE_CIRCUMFERENCE = 1.977  # meters (215/40R18 - Limited trim)

# RPM thresholds for shift suggestions
ECONOMY_UPSHIFT_RPM = 2500   # Upshift for fuel economy
ECONOMY_DOWNSHIFT_RPM = 1500  # Downshift to avoid lugging
PERFORMANCE_UPSHIFT_RPM = 6500  # Near redline upshift
PERFORMANCE_DOWNSHIFT_RPM = 4000  # Keep in powerband

# Detection thresholds
STALL_RPM_THRESHOLD = 300
LUG_RPM_THRESHOLD = 1500  # BRZ lugs noticeably below this under load
LUG_LOAD_THRESHOLD = 0.10  # 10% throttle - any meaningful load
MIN_SPEED_FOR_LUG = 1.5  # m/s (~3.4 mph)
LAUNCH_SPEED_THRESHOLD = 0.5  # m/s
LAUNCH_COMPLETE_SPEED = 5.0  # m/s

# Grading thresholds
# Acceleration std dev during shifts - measures bumpiness (lower = smoother)
GOOD_SHIFT_SMOOTHNESS = 0.5   # m/s^2 std dev - smooth shift
OK_SHIFT_SMOOTHNESS = 1.0     # m/s^2 std dev - acceptable shift
GOOD_DOWNSHIFT_REV_MATCH_TOLERANCE = 300  # RPM
OK_DOWNSHIFT_REV_MATCH_TOLERANCE = 500    # RPM
# Launch smoothness - more forgiving since accel naturally varies during launch
GOOD_LAUNCH_SMOOTHNESS = 1.5  # m/s^2 std dev - smooth clutch engagement
OK_LAUNCH_SMOOTHNESS = 2.5    # m/s^2 std dev - acceptable launch


def rpm_for_speed_and_gear(speed_ms: float, gear: int) -> float:
  """Calculate expected RPM for a given speed and gear"""
  if gear not in BRZ_GEAR_RATIOS or speed_ms <= 0:
    return 0.0
  # RPM = (speed * final_drive * gear_ratio * 60) / tire_circumference
  return (speed_ms * BRZ_FINAL_DRIVE * BRZ_GEAR_RATIOS[gear] * 60) / BRZ_TIRE_CIRCUMFERENCE


def speed_for_rpm_and_gear(rpm: float, gear: int) -> float:
  """Calculate expected speed for a given RPM and gear"""
  if gear not in BRZ_GEAR_RATIOS or rpm <= 0:
    return 0.0
  # speed = (RPM * tire_circumference) / (final_drive * gear_ratio * 60)
  return (rpm * BRZ_TIRE_CIRCUMFERENCE) / (BRZ_FINAL_DRIVE * BRZ_GEAR_RATIOS[gear] * 60)


@dataclass
class DriveSession:
  start_time: float = 0.0
  end_time: float = 0.0
  duration: float = 0.0

  stall_count: int = 0
  lug_count: int = 0
  lug_duration_total: float = 0.0

  upshift_count: int = 0
  upshift_good: int = 0
  upshift_ok: int = 0
  upshift_poor: int = 0

  downshift_count: int = 0
  downshift_good: int = 0
  downshift_ok: int = 0
  downshift_poor: int = 0

  launch_count: int = 0
  launch_good: int = 0
  launch_ok: int = 0
  launch_poor: int = 0
  launch_stalled: int = 0


@dataclass
class SessionSummary:
  """Summary of a single drive session for history tracking"""
  timestamp: float = 0.0  # Unix timestamp when session ended
  duration: float = 0.0  # seconds
  stalls: int = 0
  lugs: int = 0
  upshifts: int = 0
  upshifts_good: int = 0
  downshifts: int = 0
  downshifts_good: int = 0
  launches: int = 0
  launches_good: int = 0

  @property
  def shift_score(self) -> float:
    """Calculate shift quality score 0-100"""
    total = self.upshifts + self.downshifts
    if total == 0:
      return 100.0
    good = self.upshifts_good + self.downshifts_good
    return (good / total) * 100

  @property
  def stall_rate(self) -> float:
    """Stalls per 10 minutes of driving"""
    if self.duration < 60:
      return 0.0
    return (self.stalls / self.duration) * 600


@dataclass
class HistoricalStats:
  total_drives: int = 0
  total_drive_time: float = 0.0

  total_stalls: int = 0
  total_lugs: int = 0
  total_lug_time: float = 0.0

  total_upshifts: int = 0
  upshifts_good: int = 0
  upshifts_ok: int = 0
  upshifts_poor: int = 0

  total_downshifts: int = 0
  downshifts_good: int = 0

  # Per-gear shift quality tracking (jerk values, lower = smoother)
  # gear_shifts_into[gear] = [count, total_jerk] for averaging
  gear_shift_counts: dict = field(default_factory=lambda: {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0})
  gear_shift_jerk_totals: dict = field(default_factory=lambda: {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0})
  downshifts_ok: int = 0
  downshifts_poor: int = 0

  total_launches: int = 0
  launches_good: int = 0
  launches_ok: int = 0
  launches_poor: int = 0
  launches_stalled: int = 0

  # Per-session history for plotting (keep last 30 sessions)
  session_history: list = field(default_factory=list)

  # Legacy fields for backward compatibility
  recent_stall_rates: list = field(default_factory=list)
  recent_shift_scores: list = field(default_factory=list)


class ManualStatsTracker:
  """Singleton tracker for manual transmission driving stats"""

  _instance: Optional['ManualStatsTracker'] = None

  def __new__(cls):
    if cls._instance is None:
      cls._instance = super().__new__(cls)
      cls._instance._initialized = False
    return cls._instance

  def __init__(self):
    if self._initialized:
      return
    self._initialized = True

    self._params = Params()
    self.historical: HistoricalStats = self._load_historical_stats()

    # Frame counter - each frame is DT_CTRL (10ms)
    self.frame = 0
    self.session_start_frame = 0
    self.session: DriveSession = DriveSession(start_time=0.0)  # Will set duration from frames

    # State tracking
    self.prev_rpm = 0.0
    self.prev_gear = 0
    self.prev_speed = 0.0
    self.prev_accel = 0.0
    self.prev_throttle = 0.0

    # Acceleration history for jerk calculation (last 0.5s at ~100Hz = 50 frames)
    self.accel_history: deque = deque(maxlen=50)

    # Lugging state
    self.is_lugging = False
    self.lug_start_frame = 0

    # Launch state
    self.is_launching = False
    self.launch_start_frame = 0
    self.launch_max_jerk = 0.0
    self.launch_clutch_released = False

    # Shift validation state - require clutch press for real shifts
    self.clutch_pressed_recently = False
    self.clutch_release_frame = 0

    # Last shift info for logging to CarState
    self.last_shift_smoothness = 0.0
    self.last_shift_grade = 0  # 0=none, 1=good, 2=ok, 3=poor
    self.last_shift_frame = 0
    self.gear_before_clutch = 0  # gear when clutch was pressed

    self._live_update_counter = 0

  def _load_historical_stats(self) -> HistoricalStats:
    try:
      data = self._params.get("ManualDriveStats")
      if data:
        stats_dict = json.loads(data)
        return HistoricalStats(**stats_dict)
    except Exception:
      pass
    return HistoricalStats()

  def _save_historical_stats(self):
    """Save historical stats (non-blocking, atomic via temp file + rename)"""
    try:
      self._params.put_nonblocking("ManualDriveStats", json.dumps(asdict(self.historical)))
    except Exception:
      pass

  def _save_session_stats(self):
    """Save current session stats (non-blocking, atomic via temp file + rename)"""
    try:
      duration_seconds = (self.frame - self.session_start_frame) * DT_CTRL
      session_dict = {
        'duration': duration_seconds,
        'stall_count': self.session.stall_count,
        'lug_count': self.session.lug_count,
        'upshift_count': self.session.upshift_count,
        'upshift_good': self.session.upshift_good,
        'upshift_ok': self.session.upshift_ok,
        'upshift_poor': self.session.upshift_poor,
        'downshift_count': self.session.downshift_count,
        'downshift_good': self.session.downshift_good,
        'downshift_ok': self.session.downshift_ok,
        'downshift_poor': self.session.downshift_poor,
        'launch_count': self.session.launch_count,
        'launch_good': self.session.launch_good,
        'launch_ok': self.session.launch_ok,
        'launch_poor': self.session.launch_poor,
        'launch_stalled': self.session.launch_stalled,
        'timestamp': time.time(),
      }
      self._params.put_nonblocking("ManualDriveLastSession", json.dumps(session_dict))
    except Exception:
      pass

  def end_session(self):
    """Finalize session and update historical stats (called by UI when going offroad)"""
    self.session.duration = (self.frame - self.session_start_frame) * DT_CTRL

    # Only save sessions longer than 30 seconds
    if self.session.duration < 30:
      return

    # Update historical stats
    self.historical.total_drives += 1
    self.historical.total_drive_time += self.session.duration
    self.historical.total_stalls += self.session.stall_count
    self.historical.total_lugs += self.session.lug_count
    self.historical.total_lug_time += self.session.lug_duration_total

    self.historical.total_upshifts += self.session.upshift_count
    self.historical.upshifts_good += self.session.upshift_good
    self.historical.upshifts_ok += self.session.upshift_ok
    self.historical.upshifts_poor += self.session.upshift_poor

    self.historical.total_downshifts += self.session.downshift_count
    self.historical.downshifts_good += self.session.downshift_good
    self.historical.downshifts_ok += self.session.downshift_ok
    self.historical.downshifts_poor += self.session.downshift_poor

    self.historical.total_launches += self.session.launch_count
    self.historical.launches_good += self.session.launch_good
    self.historical.launches_ok += self.session.launch_ok
    self.historical.launches_poor += self.session.launch_poor
    self.historical.launches_stalled += self.session.launch_stalled

    # Add session to history for plotting
    summary = SessionSummary(
      timestamp=time.time(),
      duration=self.session.duration,
      stalls=self.session.stall_count,
      lugs=self.session.lug_count,
      upshifts=self.session.upshift_count,
      upshifts_good=self.session.upshift_good,
      downshifts=self.session.downshift_count,
      downshifts_good=self.session.downshift_good,
      launches=self.session.launch_count,
      launches_good=self.session.launch_good,
    )
    self.historical.session_history.append(asdict(summary))
    # Keep last 30 sessions
    if len(self.historical.session_history) > 30:
      self.historical.session_history.pop(0)

    # Legacy trend tracking (for backward compatibility)
    self.historical.recent_stall_rates.append(self.session.stall_count)
    if len(self.historical.recent_stall_rates) > 10:
      self.historical.recent_stall_rates.pop(0)

    total_shifts = self.session.upshift_count + self.session.downshift_count
    if total_shifts > 0:
      good_shifts = self.session.upshift_good + self.session.downshift_good
      ok_shifts = self.session.upshift_ok + self.session.downshift_ok
      shift_score = int((good_shifts * 100 + ok_shifts * 50) / total_shifts)
    else:
      shift_score = 100

    self.historical.recent_shift_scores.append(shift_score)
    if len(self.historical.recent_shift_scores) > 10:
      self.historical.recent_shift_scores.pop(0)

    self._save_session_stats()
    self._save_historical_stats()

  def _calculate_jerk(self) -> float:
    """
    Calculate smoothness metric from acceleration history.
    Returns the standard deviation of acceleration over the last 0.5s -
    measures how much acceleration varied during the shift.
    Lower = smoother shift.
    """
    if len(self.accel_history) < 10:
      return 0.0

    # Use last 50 frames (0.5s)
    accels = list(self.accel_history)[-50:]

    # Standard deviation of acceleration - measures bumpiness
    mean_accel = sum(accels) / len(accels)
    variance = sum((a - mean_accel) ** 2 for a in accels) / len(accels)
    std_accel = variance ** 0.5

    return std_accel

  def update(self, rpm: float, gear: int, speed: float, accel: float, clutch: bool, neutral: bool,
             throttle: float):
    """
    Main update function - called from carstate.py on each update.
    Each call represents one frame (DT_CTRL = 10ms).

    Args:
      rpm: Engine RPM
      gear: Current gear (1-6, 0 for neutral/unknown)
      speed: Vehicle speed in m/s
      accel: Vehicle acceleration in m/s^2 (from CarState.aEgo)
      clutch: True if clutch pedal is pressed
      neutral: True if in neutral
      throttle: Throttle position (0-1)
    """
    # Increment frame counter
    self.frame += 1

    # Use aEgo from CarState (Kalman filtered, more accurate than computing from speed)
    self.accel_history.append(accel)

    # "In gear" means: not in neutral AND clutch not pressed (power transmitting)
    in_gear = not neutral and not clutch

    # Track clutch state for shift validation
    # A real shift requires: clutch pressed -> gear change -> clutch released
    if clutch and not self.clutch_pressed_recently:
      # Clutch just pressed - remember the gear
      self.clutch_pressed_recently = True
      self.gear_before_clutch = self.prev_gear
    elif not clutch and self.clutch_pressed_recently:
      # Clutch just released - mark frame for shift grading window
      self.clutch_release_frame = self.frame
      self.clutch_pressed_recently = False

    # Detect stall
    self._detect_stall(rpm, clutch, neutral)

    # Detect lugging (only when in gear)
    self._detect_lug(rpm, throttle, speed, in_gear)

    # Detect launches
    self._detect_launch(rpm, speed, clutch, neutral)

    # Detect shifts (only when clutch was involved and in gear)
    self._detect_shift(rpm, gear, clutch, neutral)

    # Update previous state
    self.prev_rpm = rpm
    self.prev_gear = gear
    self.prev_speed = speed
    self.prev_accel = accel
    self.prev_throttle = throttle

    # Periodic saves (every ~500 calls = 5s at 100Hz)
    self._live_update_counter += 1
    if self._live_update_counter >= 500:
      self._live_update_counter = 0
      self._save_session_stats()
      self._update_live_stats()

  def _detect_stall(self, rpm: float, clutch: bool, neutral: bool):
    """Detect engine stall"""
    # Stall: RPM drops below threshold (edge detection)
    # Don't require clutch released - you often press clutch in panic as it stalls
    # Only require not in neutral (can't stall in neutral)
    if rpm < STALL_RPM_THRESHOLD and self.prev_rpm >= STALL_RPM_THRESHOLD:
      if not neutral:
        self.session.stall_count += 1

        # If we were launching, mark it as stalled
        if self.is_launching:
          self.session.launch_stalled += 1
          self.session.launch_count += 1
          self.is_launching = False

  def _detect_lug(self, rpm: float, throttle: float, speed: float, in_gear: bool):
    """Detect engine lugging (only when in gear - not neutral and clutch released)"""
    is_lugging_now = (
      in_gear and
      rpm < LUG_RPM_THRESHOLD and
      throttle > LUG_LOAD_THRESHOLD and
      speed > MIN_SPEED_FOR_LUG
    )

    if is_lugging_now and not self.is_lugging:
      self.is_lugging = True
      self.lug_start_frame = self.frame
      self.session.lug_count += 1
    elif not is_lugging_now and self.is_lugging:
      lug_duration = (self.frame - self.lug_start_frame) * DT_CTRL
      self.session.lug_duration_total += lug_duration
      self.is_lugging = False

  def _detect_launch(self, rpm: float, speed: float, clutch: bool, neutral: bool):
    """Detect and grade launches"""
    # Start launch when stopped with clutch pressed
    if speed < LAUNCH_SPEED_THRESHOLD and clutch and not self.is_launching:
      self.is_launching = True
      self.launch_start_frame = self.frame
      self.launch_max_jerk = 0.0
      self.launch_clutch_released = False

    if self.is_launching:
      # Track clutch release
      if not clutch:
        self.launch_clutch_released = True

      # Track max bumpiness (accel std dev)
      current_smoothness = self._calculate_jerk()
      self.launch_max_jerk = max(self.launch_max_jerk, current_smoothness)

      # Check for successful launch completion
      if speed >= LAUNCH_COMPLETE_SPEED and self.launch_clutch_released:
        # Grade the launch based on smoothness during clutch engagement
        if self.launch_max_jerk < GOOD_LAUNCH_SMOOTHNESS:
          self.session.launch_good += 1
        elif self.launch_max_jerk < OK_LAUNCH_SMOOTHNESS:
          self.session.launch_ok += 1
        else:
          self.session.launch_poor += 1

        self.session.launch_count += 1
        self.is_launching = False

  def _detect_shift(self, rpm: float, gear: int, clutch: bool, neutral: bool):
    """Detect and grade gear shifts - only counts shifts with clutch involvement"""
    if gear == 0 or neutral:
      return

    # Only grade a shift if:
    # 1. Gear changed from what it was when clutch was pressed
    # 2. Clutch was recently released (within 1 second = 100 frames)
    # 3. Not in neutral (in_gear)
    # This prevents false positives from gear prediction fluctuations
    frames_since_clutch_release = self.frame - self.clutch_release_frame
    if frames_since_clutch_release > 100:  # 100 frames = 1 second
      return  # Too long since clutch release, not a real shift

    if gear != self.gear_before_clutch and self.gear_before_clutch > 0 and not clutch:
      is_upshift = gear > self.gear_before_clutch
      smoothness = self._calculate_jerk()  # Returns accel std dev - lower is smoother

      # Track per-gear smoothness for the gear we shifted INTO
      if 1 <= gear <= 6:
        self.historical.gear_shift_counts[gear] = self.historical.gear_shift_counts.get(gear, 0) + 1
        self.historical.gear_shift_jerk_totals[gear] = self.historical.gear_shift_jerk_totals.get(gear, 0.0) + smoothness

      if is_upshift:
        # Grade upshift based on smoothness (accel std dev)
        if smoothness < GOOD_SHIFT_SMOOTHNESS:
          grade = "good"
          self.session.upshift_good += 1
        elif smoothness < OK_SHIFT_SMOOTHNESS:
          grade = "ok"
          self.session.upshift_ok += 1
        else:
          grade = "poor"
          self.session.upshift_poor += 1
        self.session.upshift_count += 1
        print(f"UPSHIFT {self.gear_before_clutch}->{gear}: smoothness={smoothness:.2f} m/s^2, grade={grade}, speed={self.prev_speed:.1f}")
      else:
        # Grade downshift - primarily on rev matching, smoothness as secondary
        # Use actual current speed (more reliable than computing from RPM during shift)
        target_rpm = rpm_for_speed_and_gear(self.prev_speed, gear)
        rpm_match_error = abs(rpm - target_rpm)

        # Good: great rev-match AND smooth
        # Ok: decent rev-match OR great rev-match but bumpy
        # Poor: bad rev-match
        if rpm_match_error < GOOD_DOWNSHIFT_REV_MATCH_TOLERANCE and smoothness < GOOD_SHIFT_SMOOTHNESS:
          grade = "good"
          self.session.downshift_good += 1
        elif rpm_match_error < OK_DOWNSHIFT_REV_MATCH_TOLERANCE or (rpm_match_error < GOOD_DOWNSHIFT_REV_MATCH_TOLERANCE and smoothness < OK_SHIFT_SMOOTHNESS):
          grade = "ok"
          self.session.downshift_ok += 1
        else:
          grade = "poor"
          self.session.downshift_poor += 1
        self.session.downshift_count += 1
        print(f"DOWNSHIFT {self.gear_before_clutch}->{gear}: smoothness={smoothness:.2f}, rpm={rpm:.0f}, target_rpm={target_rpm:.0f}, rpm_err={rpm_match_error:.0f}, speed={self.prev_speed:.1f}, grade={grade}")

      # Store shift info for CarState logging
      self.last_shift_smoothness = smoothness
      self.last_shift_grade = 1 if grade == "good" else (2 if grade == "ok" else 3)
      self.last_shift_frame = self.frame

      # Reset so we don't double-count this shift
      self.clutch_release_frame = 0
      self.gear_before_clutch = 0

  def predict_gear(self, rpm: float, speed_ms: float) -> int:
    """
    Predict current gear from RPM and speed using BRZ gear ratios.
    Returns gear 1-6, or 0 if unable to determine (stopped, neutral, etc.)
    """
    if rpm < 500 or speed_ms < 0.5:
      return 0  # Engine off, idling, or stopped

    # Calculate expected RPM for each gear at current speed
    best_gear = 0
    min_error = float('inf')

    for gear in range(1, 7):
      expected_rpm = rpm_for_speed_and_gear(speed_ms, gear)
      error = abs(rpm - expected_rpm)

      # Weight lower gears slightly to handle clutch slip during acceleration
      if gear <= 2:
        error *= 0.9

      if error < min_error:
        min_error = error
        best_gear = gear

    # Only return gear if error is reasonable (within 500 RPM)
    # This handles clutch slip, coasting, etc.
    if min_error < 500:
      return best_gear
    return 0

  def get_shift_suggestion(self, rpm: float, gear: int, throttle: float) -> dict:
    """
    Get shift suggestion based on current driving conditions.
    Returns dict with 'action' ('upshift', 'downshift', 'ok') and 'reason'.
    """
    if gear == 0 or rpm < 500:
      return {'action': 'ok', 'reason': '', 'target_gear': 0}

    # Economy mode (light throttle / cruising)
    if throttle < 0.3:
      if rpm > ECONOMY_UPSHIFT_RPM and gear < 6:
        return {
          'action': 'upshift',
          'reason': 'economy',
          'target_gear': gear + 1,
          'target_rpm': int(rpm_for_speed_and_gear(speed_for_rpm_and_gear(rpm, gear), gear + 1))
        }
      if rpm < ECONOMY_DOWNSHIFT_RPM and gear > 1:
        return {
          'action': 'downshift',
          'reason': 'lug_prevention',
          'target_gear': gear - 1,
          'target_rpm': int(rpm_for_speed_and_gear(speed_for_rpm_and_gear(rpm, gear), gear - 1))
        }

    # Performance mode (heavy throttle)
    if throttle > 0.7:
      if rpm > PERFORMANCE_UPSHIFT_RPM and gear < 6:
        return {
          'action': 'upshift',
          'reason': 'redline',
          'target_gear': gear + 1,
          'target_rpm': int(rpm_for_speed_and_gear(speed_for_rpm_and_gear(rpm, gear), gear + 1))
        }
      if rpm < PERFORMANCE_DOWNSHIFT_RPM and gear > 1:
        return {
          'action': 'downshift',
          'reason': 'powerband',
          'target_gear': gear - 1,
          'target_rpm': int(rpm_for_speed_and_gear(speed_for_rpm_and_gear(rpm, gear), gear - 1))
        }

    return {'action': 'ok', 'reason': '', 'target_gear': 0}

  def get_rev_match_target(self, current_gear: int, target_gear: int, speed_ms: float) -> int:
    """Calculate target RPM for rev-matching a downshift"""
    if target_gear not in BRZ_GEAR_RATIOS:
      return 0
    return int(rpm_for_speed_and_gear(speed_ms, target_gear))

  def get_shift_info(self) -> tuple:
    """Get last shift info for CarState logging. Returns (smoothness, grade).
    Values are only non-zero for 50 frames (0.5s) after a shift."""
    if self.frame - self.last_shift_frame < 50:
      return (self.last_shift_smoothness, self.last_shift_grade)
    return (0.0, 0)

  def get_live_stats(self) -> dict:
    """Get current session stats for live display"""
    # Get shift suggestion
    suggestion = self.get_shift_suggestion(self.prev_rpm, self.prev_gear, self.prev_throttle)

    return {
      'stalls': self.session.stall_count,
      'lugs': self.session.lug_count,
      'shifts': self.session.upshift_count + self.session.downshift_count,
      'good_shifts': self.session.upshift_good + self.session.downshift_good,
      'launches': self.session.launch_count,
      'good_launches': self.session.launch_good,
      'is_lugging': self.is_lugging,
      'gear': self.prev_gear,
      'shift_suggestion': suggestion['action'],
      'shift_reason': suggestion['reason'],
    }

  def _update_live_stats(self):
    """Write live stats to Params for UI display"""
    try:
      self._params.put_nonblocking("ManualDriveLiveStats", self.get_live_stats())
    except Exception:
      pass


# Global instance
_tracker: Optional[ManualStatsTracker] = None


def get_tracker() -> ManualStatsTracker:
  """Get the singleton tracker instance"""
  global _tracker
  if _tracker is None:
    _tracker = ManualStatsTracker()
  return _tracker
