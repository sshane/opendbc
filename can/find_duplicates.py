#!/usr/bin/env python3
from __future__ import print_function
import os
import re

from collections import defaultdict
from opendbc.can.dbc import dbc  # TODO: use this
from opendbc.generator.generator import read_dbc

cur_path = os.path.dirname(os.path.realpath(__file__))
opendbc_root = os.path.join(cur_path, '../')


class Message:
  def __init__(self, title):
    self.title = title
    self.name = self.title.split(' ')[1]
    self.addr = self.title.split(' ')[0]

  def __repr__(self):
    return self.title


def is_message(line):
  return line.startswith("BO_ ")


honda_dir = os.path.join(opendbc_root, 'generator', 'honda')

if __name__ == "__main__":
  common_honda_msgs = [line for line in read_dbc(honda_dir, "_honda_common.dbc").splitlines() if is_message(line)]
  common_honda_msgs += [line for line in read_dbc(honda_dir, "_comma.dbc").splitlines() if is_message(line)]

  dbc_messages = defaultdict(list)
  len_orig_dbcs = 0
  for dbc_fn in os.listdir(opendbc_root):
    if not dbc_fn.startswith("honda") or (dbc_fn.startswith('_') or not dbc_fn.endswith('.dbc')):
      continue
    len_orig_dbcs += 1
    dbc_file_in = read_dbc(opendbc_root, dbc_fn)
    for line in dbc_file_in.splitlines():
      if is_message(line):
        line = ' '.join(line.split(' ')[1:-1])
        dbc_messages[dbc_fn].append(Message(line))
    # print(dbc_file_in)
  # print(dbc_messages['_honda_2017.dbc'])

  common_msgs = defaultdict(set)
  for dbc_fn, msgs in dbc_messages.items():
    for dbc_fn_2, msgs_2 in dbc_messages.items():
      if dbc_fn == dbc_fn_2:
        continue
      for msg in msgs:
        duplicate_msg = msg.title in [s.title for s in msgs_2]
        msg_exists = msg.name in [s.name for s in msgs_2] or msg.addr in [s.addr for s in msgs_2]
        if duplicate_msg or not msg_exists:  # if duplicate or doesn't exist (okay to create superset)
          common_msgs[msg.title].add(dbc_fn)
          common_msgs[msg.title].add(dbc_fn_2)
  common_msgs = {msg: tuple(dbcs) for msg, dbcs in common_msgs.items()}

  for msg, dbcs in common_msgs.items():
    if len(dbcs) == len_orig_dbcs and msg not in common_honda_msgs:
      print('Message common across all DBCs: {}'.format(msg))

  raise Exception

  print(len(common_msgs))
  # print(common_sigs)
  print()

  new_dbcs = {}
  # new_dbcs = []
  duplicates = 0
  for msg, dbcs in common_msgs.items():
    for msg_2, dbcs_2 in common_msgs.items():
      if msg == msg_2:
        continue
      if dbcs == dbcs_2:
        duplicates += 1
        new_dbcs.append(dbcs)
    # print(msg, dbcs)
  print(duplicates)
