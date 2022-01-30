#!/usr/bin/env python3
from __future__ import print_function
import os
import re

from collections import defaultdict
from opendbc.can.dbc import dbc as dbc_helper  # TODO: use this
from opendbc.generator.generator import read_dbc

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
    # TODO: take into account signal start bits
    sig_check = all([sig_start_bit == other.sigs[sig_name] for sig_name, sig_start_bit in
                     self.sigs.items() if sig_name in other.sigs])
    # print(self.title, sig_check)
    return self.title == other.title and sig_check

  def __hash__(self):
    return hash(self.title)

  def __repr__(self):
    return self.title


def is_message(line):
  return line.startswith("BO_ ")

def msgs_equal(msg1, msg2):
  name_check = msg1[0][0] == msg2[0][0]
  len_check = msg1[0][1] == msg2[0][1]
  return sig1 == sig2


platform = "toyota"
platform_dir = os.path.join(opendbc_root, 'generator', platform)
common_dbc = os.path.join(platform_dir, "_comma.dbc")

if __name__ == "__main__":
  common_comma_msgs = []
  if os.path.exists(common_dbc):
    common_comma_msgs = [Message(addr, msg) for addr, msg in dbc_helper(common_dbc).msgs.items()]
  # common_honda_msgs = [Message(addr, msg).title for addr, msg in dbc_helper(os.path.join(honda_dir, "_comma.dbc")).msgs.items()]
  # print(common_honda_msgs)

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


  # print(common_sigs)

  # first create base DBC that is a superset of all common signals
  base_dbc = []  # holds messages
  for msg, dbcs in common_msgs.items():
    if len(dbcs) == len_orig_dbcs:
      base_dbc.append(msg)
      # # common here (as above) means either duplicate, or it doesn't conflict with another address or name
      # print('Message common across all DBCs: {}'.format(msg))
  # print('Base DBC (len: {}): {}'.format(len(base_dbc), base_dbc))

  new_dbcs = defaultdict(set)

  for msg, dbcs in common_msgs.items():
    if msg in base_dbc:
      continue
    if len(dbcs) == 1:  # message isn't common with anything, need a new DBC :(
      # print(msg, dbcs)
      new_dbcs[list(dbcs)[0]].add(msg)

  print('Need a minimum of {} DBCs'.format(len(new_dbcs)))
  # raise Exception

  common_msgs_to_add = {msg: dbcs for msg, dbcs in common_msgs.items() if msg not in base_dbc}
  print(common_msgs_to_add)
  print(new_dbcs)
  print('\n')
  # raise Exception

  for msg, dbcs in common_msgs_to_add.items():
    for dbc in dbcs:
      print('adding {} to {}'.format(msg, dbc))
      new_dbcs[dbc].add(msg)

  # raise Exception

  # for msg, dbcs in common_msgs_to_add.items():
  #   # if msg.name == "BRAKE_MODULE":
  #   #   continue
  #   # print(msg, base_dbc)
  #   # print(type(dbcs))
  #   added_to_a_dbc = False
  #   for new_dbc, new_dbc_msgs in new_dbcs.items():
  #     good_to_go = True
  #     for other_msg in new_dbc_msgs:
  #       # if msg.name == "EPS_STATUS":
  #       print('new dbc: {}'.format(new_dbc))
  #       print("this: {} {}, other: {} {}, ={}".format(msg, dbcs, other_msg, messages_to_dbc[other_msg], all([this_dbc in messages_to_dbc[other_msg] for this_dbc in dbcs])))
  #       good_to_go = good_to_go and all([this_dbc in messages_to_dbc[other_msg] for this_dbc in dbcs])
  #
  #     # checked all msgs in new prospective DBC
  #     if good_to_go:
  #       # if msg.name in ["EPS_STATUS", ]:  # EPS_STATUS
  #       print('adding {} to {}'.format(msg, new_dbc))
  #       new_dbcs[new_dbc].add(msg)
  #       added_to_a_dbc = True
  #       # break
  #   if not added_to_a_dbc:
  #     print('making new DBC for {} (was adding to {})'.format(msg, new_dbc))
  #     new_dbcs[f"new_dbc{len(new_dbcs)}"].add(msg)
  #   print()

  new_dbcs = dict(new_dbcs)
  # print(len)
  # raise Exception
  #
  # Perform sanity check
  in_new_dbcs = True
  for msg, dbcs in messages_to_dbc.items():
    if msg in base_dbc:
      continue
    print(msg)
    for new_dbc_msgs in new_dbcs.values():
      print(type(msg), len(new_dbc_msgs))
      in_new_dbcs = in_new_dbcs and msg in new_dbc_msgs
  assert in_new_dbcs, "Not all messages are in the new DBCs"
  #
  # raise Exception
  #
  # new_dbcs = {}
  # # new_dbcs = []
  # duplicates = 0
  # for msg, this_dbcs in common_msgs.items():
  #   if len(new_dbcs) == 0:
  #     new_dbcs[f'dbc{len(new_dbcs)}'] = [msg]
  #     print("HERE")
  #     continue
  #
  #   in_any = False
  #   for new_dbc, new_dbc_msgs in new_dbcs.items():
  #     all_in = True
  #     for new_dbc_msg in new_dbc_msgs:
  #       all_in = all_in and all([dbc in this_dbcs for dbc in common_msgs[new_dbc_msg]])
  #     print(all_in)
  #     if all_in:
  #       new_dbcs[new_dbc].append(msg)
  #       in_any = True
  #       break
  #
  #   if not in_any:
  #     new_dbcs[f'dbc{len(new_dbcs)}'] = [msg]
  #
  #     # print(new_dbc_msgs)
  #     # for this_msg_dbc in this_dbcs:
  #     #   all_in = all_in and this_msg_dbc in this_dbcs
  #     # print(all_in)
  #
  # #   for msg_2, dbcs_2 in common_msgs.items():
  # #     if msg == msg_2:
  # #       continue
  # #     if dbcs == dbcs_2:
  # #       duplicates += 1
  # #       new_dbcs.append(dbcs)
  # #   # print(msg, dbcs)
  # # print(duplicates)
