#!/usr/bin/env python3
from __future__ import print_function
import os

from collections import defaultdict
from opendbc.can.dbc import dbc as dbc_helper  # TODO: use this

cur_path = os.path.dirname(os.path.realpath(__file__))
opendbc_root = os.path.join(cur_path, '../')


class Message:
  def __init__(self, addr, info):
    self.name = info[0][0]
    self.len = info[0][1]
    self.addr = addr
    self.sigs = {sig.name: sig.start_bit for sig in info[1]}
    self.title = f"{addr} {self.name}: {self.len}"  # addr, name, msg length. quick and dirty check

  def __eq__(self, other):
    sig_check = all([sig_start_bit == other.sigs[sig_name] for sig_name, sig_start_bit in
                     self.sigs.items() if sig_name in other.sigs])
    return self.title == other.title and sig_check

  def __hash__(self):
    return hash(self.title)

  def __repr__(self):
    return self.title


def get_base_dbcs(_common_msgs, _len_orig_dbcs):
  # create base DBCs that are supersets of all common signals
  base_dbcs = []  # list of lists of messages
  n = _len_orig_dbcs
  while n >= 0:
    print('n', n)
    print('len', len(base_dbcs))
    msgs = [msg for msg, dbcs in _common_msgs.items() if len(dbcs) == n]
    if len(msgs):
      base_dbcs.append(msgs)
    n -= 1
  return base_dbcs


platform = "honda"
platform_dir = os.path.join(opendbc_root, 'generator', platform)
common_dbc = os.path.join(platform_dir, "_comma.dbc")

if __name__ == "__main__":
  common_comma_msgs = []
  if os.path.exists(common_dbc):
    common_comma_msgs = [Message(addr, msg) for addr, msg in dbc_helper(common_dbc).msgs.items()]

  dbc_to_messages = {}
  messages_to_dbc = defaultdict(set)
  len_orig_dbcs = 0
  for dbc_fn in os.listdir(opendbc_root):
    if not dbc_fn.startswith(platform) or dbc_fn.startswith('_') or not dbc_fn.endswith('generated.dbc'):
      continue
    print(dbc_fn)
    len_orig_dbcs += 1
    dbc = dbc_helper(os.path.join(opendbc_root, dbc_fn))
    msgs = [Message(addr, msg) for addr, msg in dbc_helper(os.path.join(opendbc_root, dbc_fn)).msgs.items()]
    # msgs = [msg for msg in msgs if msg not in defined_common_msgs]
    dbc_to_messages[dbc_fn] = msgs
    for msg in msgs:
      messages_to_dbc[msg].add(dbc_fn)
  messages_to_dbc = dict(messages_to_dbc)
  print()

  # dict map of msg to all dbc's that contain it without conflict (either exists or doesn't with no conflicts)
  common_msgs = defaultdict(set)
  for dbc_fn, msgs in dbc_to_messages.items():
    for dbc_fn_2, msgs_2 in dbc_to_messages.items():
      if dbc_fn == dbc_fn_2:
        continue
      for msg in msgs:
        if msg in common_comma_msgs:
          continue
        common_msgs[msg].add(dbc_fn)
        duplicate_msg = msg.title in [s.title for s in msgs_2]
        msg_exists = msg.name in [s.name for s in msgs_2] or msg.addr in [s.addr for s in msgs_2]
        if duplicate_msg or not msg_exists:  # if duplicate or doesn't exist (okay to create superset)
          # print('adding', msg)
          common_msgs[msg].add(dbc_fn_2)

  # messages_to_dbc contains all (even defined common),
  # common_msgs excludes common defined but includes DBCs that don't have any conflicts for including that message
  common_msgs = dict(common_msgs)

  # first create base DBC that is a superset of all common signals
  base_dbc = get_base_dbcs(common_msgs, len_orig_dbcs)[0]

  new_dbcs = defaultdict(set)

  for msg, dbcs in common_msgs.items():
    if msg in base_dbc:
      continue
    if len(dbcs) == 1:  # message isn't common with anything, need a new DBC :(
      new_dbcs[list(dbcs)[0]].add(msg)

  print('Need a minimum of {} DBCs'.format(len(new_dbcs)))

  common_msgs_to_add = {msg: dbcs for msg, dbcs in common_msgs.items() if msg not in base_dbc}
  print(common_msgs_to_add)
  print(new_dbcs)
  print()

  for msg, dbcs in common_msgs_to_add.items():
    for dbc in dbcs:
      print('adding {} to {}'.format(msg, dbc))
      new_dbcs[dbc].add(msg)

  new_dbcs = dict(new_dbcs)
  print('\nCollapsed to {} DBCs'.format(len(new_dbcs)))
  print('1 base DBC with {} messages'.format(len(base_dbc)))

  # # Perform sanity check
  # in_new_dbcs = True
  # for msg, dbcs in messages_to_dbc.items():
  #   if msg in base_dbc:
  #     continue
  #   print(msg)
  #   for new_dbc_msgs in new_dbcs.values():
  #     print(type(msg), len(new_dbc_msgs))
  #     in_new_dbcs = in_new_dbcs and msg in new_dbc_msgs
  # assert in_new_dbcs, "Not all messages are in the new DBCs"
