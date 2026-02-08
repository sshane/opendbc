"""
Manual Transmission Driving Stats Tracker

Tracks and analyzes manual driving behavior:
- Stalls: Engine dies while trying to move
- Lugging: Driving at too-low RPM under load
- Shift quality: How well upshifts and downshifts are executed
- Launch smoothness: How smoothly the car starts from a stop
- Gear prediction from RPM and speed
- Shift suggestions for economy/performance

All stats are accumulated into one dict (self.stats) and saved periodically.
Current drive stats are computed as diffs from a snapshot taken at init.
No end_session() needed - card process gets killed, data is already saved.

Called from Subaru carstate.py on each update.
"""

import math
import time
from collections import deque

from openpilot.common.params import Params
from openpilot.common.time_helpers import system_time_valid
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
ECONOMY_UPSHIFT_RPM = 3200   # BRZ feels good shifting here for economy
ECONOMY_DOWNSHIFT_RPM = 1500  # Downshift to avoid lugging
PERFORMANCE_UPSHIFT_RPM = 6500  # Near redline upshift
PERFORMANCE_DOWNSHIFT_RPM = 4000  # Keep in powerband

# Detection thresholds
STALL_RPM_THRESHOLD = 300
LUG_RPM_THRESHOLD = 1200  # BRZ actually lugs below ~1200 under meaningful load
LUG_LOAD_THRESHOLD = 0.25  # 25% throttle - real load, not just maintaining speed
MIN_SPEED_FOR_LUG = 5.0  # m/s (~11 mph) - ignore very low speed creeping
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

# Save intervals (in frames, each frame = DT_CTRL = 10ms)
LIVE_SAVE_INTERVAL = 500    # 5s - for onroad widget
FULL_SAVE_INTERVAL = 1500   # 15s - for historical + session

# Keys in the stats dict that are counters (for drive snapshot diffing)
COUNTER_KEYS = [
  'total_stalls', 'total_lugs',
  'total_upshifts', 'upshifts_good', 'upshifts_ok', 'upshifts_poor',
  'total_downshifts', 'downshifts_good', 'downshifts_ok', 'downshifts_poor',
  'total_launches', 'launches_good', 'launches_ok', 'launches_poor', 'launches_stalled',
]


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


def _default_stats() -> dict:
  """Default stats dict with all keys initialized"""
  return {
    'total_drives': 0,
    'total_drive_time': 0.0,
    'total_stalls': 0,
    'total_lugs': 0,
    'total_lug_time': 0.0,
    'total_upshifts': 0,
    'upshifts_good': 0,
    'upshifts_ok': 0,
    'upshifts_poor': 0,
    'total_downshifts': 0,
    'downshifts_good': 0,
    'downshifts_ok': 0,
    'downshifts_poor': 0,
    'total_launches': 0,
    'launches_good': 0,
    'launches_ok': 0,
    'launches_poor': 0,
    'launches_stalled': 0,
    # Per-gear shift quality tracking
    'gear_shift_counts': {str(g): 0 for g in range(1, 7)},
    'gear_shift_jerk_totals': {str(g): 0.0 for g in range(1, 7)},
    # Per-session history for plotting (keep last 30 sessions)
    'session_history': [],
  }


class ManualStatsTracker:
  """Tracker for manual transmission driving stats.

  All stats accumulate directly into self.stats (a flat dict).
  Current drive counters are computed by diffing against a snapshot taken at init.
  Saved periodically to Params -- no end_session needed.
  """

  def __init__(self):
    self._params = Params()
    self.stats: dict = self._load()

    # This is a new drive -- bump drive count once
    self.stats['total_drives'] = self.stats.get('total_drives', 0) + 1

    # Snapshot counters at drive start for computing current drive stats
    self._drive_start: dict = {k: self.stats.get(k, 0) for k in COUNTER_KEYS}
    self._drive_start_lug_time: float = self.stats.get('total_lug_time', 0.0)
    self._drive_start_time: float = self.stats.get('total_drive_time', 0.0)
    self._session_history_idx: int | None = None  # Index of this drive's entry in session_history

    # Frame counter - each frame is DT_CTRL (10ms)
    self.frame = 0

    # State tracking
    self.prev_rpm = 0.0
    self.prev_gear = 0
    self.prev_speed = 0.0
    self.prev_accel = 0.0
    self.prev_throttle = 0.0

    # Acceleration history for smoothness calculation (last 0.5s at ~100Hz = 50 frames)
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

    self._live_counter = 0
    self._full_save_counter = 0

  def _load(self) -> dict:
    """Load stats from Params, or return defaults"""
    data = self._params.get("ManualDriveStats")
    if data and isinstance(data, dict):
      # Merge with defaults to ensure all keys exist
      defaults = _default_stats()
      defaults.update(data)
      return defaults
    return _default_stats()

  def _save(self):
    """Save all stats to Params (non-blocking, atomic)"""
    # Update drive time from frame count
    self.stats['total_drive_time'] = self._drive_start_time + self.frame * DT_CTRL

    # Update current session in session_history (replace last entry for this drive)
    drive = self.current_drive()
    session_entry = {
      'timestamp': time.time() if system_time_valid() else math.nan,
      'duration': self.frame * DT_CTRL,
      'stalls': drive['stall_count'],
      'lugs': drive['lug_count'],
      'upshifts': drive['upshift_count'],
      'upshifts_good': drive['upshift_good'],
      'downshifts': drive['downshift_count'],
      'downshifts_good': drive['downshift_good'],
      'launches': drive['launch_count'],
      'launches_good': drive['launch_good'],
      'launches_stalled': drive['launch_stalled'],
    }
    history = self.stats.get('session_history', [])
    if self._session_history_idx is not None:
      history[self._session_history_idx] = session_entry
    else:
      history.append(session_entry)
      self._session_history_idx = len(history) - 1
    # Keep last 30 sessions
    if len(history) > 30:
      history.pop(0)
      self._session_history_idx = len(history) - 1
    self.stats['session_history'] = history

    self._params.put_nonblocking("ManualDriveStats", self.stats)

  def _update_live_stats(self):
    """Write live stats to Params for the onroad widget"""
    self._params.put_nonblocking("ManualDriveLiveStats", self.get_live_stats())

  def current_drive(self) -> dict:
    """Get current drive's stats by diffing against drive start snapshot"""
    return {
      'stall_count': self.stats.get('total_stalls', 0) - self._drive_start.get('total_stalls', 0),
      'lug_count': self.stats.get('total_lugs', 0) - self._drive_start.get('total_lugs', 0),
      'upshift_count': self.stats.get('total_upshifts', 0) - self._drive_start.get('total_upshifts', 0),
      'upshift_good': self.stats.get('upshifts_good', 0) - self._drive_start.get('upshifts_good', 0),
      'upshift_ok': self.stats.get('upshifts_ok', 0) - self._drive_start.get('upshifts_ok', 0),
      'upshift_poor': self.stats.get('upshifts_poor', 0) - self._drive_start.get('upshifts_poor', 0),
      'downshift_count': self.stats.get('total_downshifts', 0) - self._drive_start.get('total_downshifts', 0),
      'downshift_good': self.stats.get('downshifts_good', 0) - self._drive_start.get('downshifts_good', 0),
      'downshift_ok': self.stats.get('downshifts_ok', 0) - self._drive_start.get('downshifts_ok', 0),
      'downshift_poor': self.stats.get('downshifts_poor', 0) - self._drive_start.get('downshifts_poor', 0),
      'launch_count': self.stats.get('total_launches', 0) - self._drive_start.get('total_launches', 0),
      'launch_good': self.stats.get('launches_good', 0) - self._drive_start.get('launches_good', 0),
      'launch_ok': self.stats.get('launches_ok', 0) - self._drive_start.get('launches_ok', 0),
      'launch_poor': self.stats.get('launches_poor', 0) - self._drive_start.get('launches_poor', 0),
      'launch_stalled': self.stats.get('launches_stalled', 0) - self._drive_start.get('launches_stalled', 0),
    }

  def _calculate_smoothness(self) -> float:
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
    return variance ** 0.5

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
    self.frame += 1
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

    # Detection
    self._detect_stall(rpm, clutch, neutral)
    self._detect_lug(rpm, throttle, speed, in_gear)
    self._detect_launch(rpm, speed, clutch, neutral)
    self._detect_shift(rpm, gear, clutch, neutral)

    # Update previous state
    self.prev_rpm = rpm
    self.prev_gear = gear
    self.prev_speed = speed
    self.prev_accel = accel
    self.prev_throttle = throttle

    # Periodic live stats (every 5s)
    self._live_counter += 1
    if self._live_counter >= LIVE_SAVE_INTERVAL:
      self._live_counter = 0
      self._update_live_stats()

    # Periodic full save (every 30s)
    self._full_save_counter += 1
    if self._full_save_counter >= FULL_SAVE_INTERVAL:
      self._full_save_counter = 0
      self._save()

  def _detect_stall(self, rpm: float, clutch: bool, neutral: bool):
    """Detect engine stall - RPM drops below threshold (edge detection)"""
    if rpm < STALL_RPM_THRESHOLD and self.prev_rpm >= STALL_RPM_THRESHOLD:
      if not neutral:
        self.stats['total_stalls'] += 1

        # Save immediately so stall isn't lost if card gets killed
        self._save()

        if self.is_launching:
          self.stats['launches_stalled'] += 1
          self.stats['total_launches'] += 1
          self.is_launching = False

  def _detect_lug(self, rpm: float, throttle: float, speed: float, in_gear: bool):
    """Detect engine lugging (only when in gear)"""
    is_lugging_now = (
      in_gear and
      rpm < LUG_RPM_THRESHOLD and
      throttle > LUG_LOAD_THRESHOLD and
      speed > MIN_SPEED_FOR_LUG
    )

    if is_lugging_now and not self.is_lugging:
      self.is_lugging = True
      self.lug_start_frame = self.frame
      self.stats['total_lugs'] += 1
    elif not is_lugging_now and self.is_lugging:
      lug_duration = (self.frame - self.lug_start_frame) * DT_CTRL
      self.stats['total_lug_time'] = self.stats.get('total_lug_time', 0.0) + lug_duration
      self.is_lugging = False

  def _detect_launch(self, rpm: float, speed: float, clutch: bool, neutral: bool):
    """Detect and grade launches"""
    if speed < LAUNCH_SPEED_THRESHOLD and clutch and not self.is_launching:
      self.is_launching = True
      self.launch_start_frame = self.frame
      self.launch_max_jerk = 0.0
      self.launch_clutch_released = False

    if self.is_launching:
      if not clutch:
        self.launch_clutch_released = True

      current_smoothness = self._calculate_smoothness()
      self.launch_max_jerk = max(self.launch_max_jerk, current_smoothness)

      if speed >= LAUNCH_COMPLETE_SPEED and self.launch_clutch_released:
        if self.launch_max_jerk < GOOD_LAUNCH_SMOOTHNESS:
          self.stats['launches_good'] += 1
        elif self.launch_max_jerk < OK_LAUNCH_SMOOTHNESS:
          self.stats['launches_ok'] += 1
        else:
          self.stats['launches_poor'] += 1

        self.stats['total_launches'] += 1
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
    if frames_since_clutch_release > 100:  # 1 second
      return

    if gear != self.gear_before_clutch and self.gear_before_clutch > 0 and not clutch:
      is_upshift = gear > self.gear_before_clutch
      smoothness = self._calculate_smoothness()

      # Track per-gear smoothness
      gear_key = str(gear)
      counts = self.stats.get('gear_shift_counts', {})
      jerks = self.stats.get('gear_shift_jerk_totals', {})
      counts[gear_key] = counts.get(gear_key, 0) + 1
      jerks[gear_key] = jerks.get(gear_key, 0.0) + smoothness
      self.stats['gear_shift_counts'] = counts
      self.stats['gear_shift_jerk_totals'] = jerks

      if is_upshift:
        if smoothness < GOOD_SHIFT_SMOOTHNESS:
          grade = "good"
          self.stats['upshifts_good'] += 1
        elif smoothness < OK_SHIFT_SMOOTHNESS:
          grade = "ok"
          self.stats['upshifts_ok'] += 1
        else:
          grade = "poor"
          self.stats['upshifts_poor'] += 1
        self.stats['total_upshifts'] += 1
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
          self.stats['downshifts_good'] += 1
        elif rpm_match_error < OK_DOWNSHIFT_REV_MATCH_TOLERANCE or (rpm_match_error < GOOD_DOWNSHIFT_REV_MATCH_TOLERANCE and smoothness < OK_SHIFT_SMOOTHNESS):
          grade = "ok"
          self.stats['downshifts_ok'] += 1
        else:
          grade = "poor"
          self.stats['downshifts_poor'] += 1
        self.stats['total_downshifts'] += 1
        print(f"DOWNSHIFT {self.gear_before_clutch}->{gear}: smoothness={smoothness:.2f}, rpm={rpm:.0f}, target_rpm={target_rpm:.0f}, rpm_err={rpm_match_error:.0f}, speed={self.prev_speed:.1f}, grade={grade}")

      # Store shift info for CarState logging
      self.last_shift_smoothness = smoothness
      self.last_shift_grade = 1 if grade == "good" else (2 if grade == "ok" else 3)
      self.last_shift_frame = self.frame

      # Reset so we don't double-count
      self.clutch_release_frame = 0
      self.gear_before_clutch = 0

  def predict_gear(self, rpm: float, speed_ms: float) -> int:
    """Predict current gear from RPM and speed using BRZ gear ratios."""
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
    """Get current drive stats for the onroad widget"""
    drive = self.current_drive()
    suggestion = self.get_shift_suggestion(self.prev_rpm, self.prev_gear, self.prev_throttle)

    return {
      'stalls': drive['stall_count'],
      'lugs': drive['lug_count'],
      'shifts': drive['upshift_count'] + drive['downshift_count'],
      'good_shifts': drive['upshift_good'] + drive['downshift_good'],
      'launches': drive['launch_count'],
      'good_launches': drive['launch_good'],
      'is_lugging': self.is_lugging,
      'is_launching': self.is_launching,
      'gear': self.prev_gear,
      'shift_suggestion': suggestion['action'],
      'shift_reason': suggestion['reason'],
    }


# Module-level singleton
_tracker = None


def get_tracker() -> ManualStatsTracker:
  """Get the singleton tracker instance"""
  global _tracker
  if _tracker is None:
    _tracker = ManualStatsTracker()
  return _tracker
