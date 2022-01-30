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
    self.title = f"{addr} {info[0][0]} {info[0][1]}"  # addr, name, msg length
    self.info = info
    self.name = info[0][0]
    self.addr = addr

  def __eq__(self, other):
    name_check = self.info[0][0] == other[0][0]
    len_check = self.info[0][1] == other[0][1]
    return name_check and len_check

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


honda_dir = os.path.join(opendbc_root, 'generator', 'honda')

if __name__ == "__main__":
  # common_honda_msgs = [Message(addr, msg).title for addr, msg in dbc_helper(os.path.join(honda_dir, "_honda_common.dbc")).msgs.items()]
  common_honda_msgs = [Message(addr, msg).title for addr, msg in dbc_helper(os.path.join(honda_dir, "_comma.dbc")).msgs.items()]
  print(common_honda_msgs)

  dbc_to_messages = {}
  messages_to_dbc = defaultdict(set)
  len_orig_dbcs = 0
  for dbc_fn in os.listdir(opendbc_root):
    if not dbc_fn.startswith("honda") or (dbc_fn.startswith('_') or not dbc_fn.endswith('.dbc')):
      continue
    len_orig_dbcs += 1
    dbc = dbc_helper(os.path.join(opendbc_root, dbc_fn))
    msgs = [Message(addr, msg) for addr, msg in dbc_helper(os.path.join(opendbc_root, dbc_fn)).msgs.items()]
    dbc_to_messages[dbc_fn] = msgs
  print(dbc.msgs)

  # dict map of msg to all dbc's that contain it without conflict
  common_msgs = defaultdict(set)
  for dbc_fn, msgs in dbc_to_messages.items():
    for dbc_fn_2, msgs_2 in dbc_to_messages.items():
      if dbc_fn == dbc_fn_2:
        continue
      for msg in msgs:
        duplicate_msg = msg.title in [s.title for s in msgs_2]
        msg_exists = msg.name in [s.name for s in msgs_2] or msg.addr in [s.addr for s in msgs_2]
        if (duplicate_msg or not msg_exists) and msg.title not in common_honda_msgs:  # if duplicate or doesn't exist (okay to create superset)
          common_msgs[msg.title].add(dbc_fn)
          common_msgs[msg.title].add(dbc_fn_2)
  common_msgs = {msg: tuple(dbcs) for msg, dbcs in common_msgs.items()}

  # for msg, dbcs in common_msgs.items():
  #   if len(dbcs) == len_orig_dbcs:
  #     # common here (as above) means either duplicate, or it doesn't conflict with another address or name
  #     print('Message common across all DBCs: {}'.format(msg))


  print(len(common_msgs))
  # print(common_sigs)
  print()

  new_dbcs = {}
  # new_dbcs = []
  duplicates = 0
  for msg, this_dbcs in common_msgs.items():
    if len(new_dbcs) == 0:
      new_dbcs[f'dbc{len(new_dbcs)}'] = [msg]
      print("HERE")
      continue

    in_any = False
    for new_dbc, new_dbc_msgs in new_dbcs.items():
      all_in = True
      for new_dbc_msg in new_dbc_msgs:
        all_in = all_in and all([dbc in this_dbcs for dbc in common_msgs[new_dbc_msg]])
      print(all_in)
      if all_in:
        new_dbcs[new_dbc].append(msg)
        in_any = True
        break

    if not in_any:
      new_dbcs[f'dbc{len(new_dbcs)}'] = [msg]

      # print(new_dbc_msgs)
      # for this_msg_dbc in this_dbcs:
      #   all_in = all_in and this_msg_dbc in this_dbcs
      # print(all_in)

  #   for msg_2, dbcs_2 in common_msgs.items():
  #     if msg == msg_2:
  #       continue
  #     if dbcs == dbcs_2:
  #       duplicates += 1
  #       new_dbcs.append(dbcs)
  #   # print(msg, dbcs)
  # print(duplicates)
