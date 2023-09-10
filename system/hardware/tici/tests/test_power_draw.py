#!/usr/bin/env python3
import unittest
import time
import math
import threading
import numpy as np
from dataclasses import dataclass
from tabulate import tabulate
from typing import List

import cereal.messaging as messaging
from cereal.services import service_list
from openpilot.system.hardware import HARDWARE, TICI
from openpilot.system.hardware.tici.power_monitor import get_power, wait_for_power
from openpilot.selfdrive.manager.process_config import managed_processes
from openpilot.selfdrive.manager.manager import manager_cleanup
from openpilot.selfdrive.navd.tests.test_map_renderer import gen_llk


@dataclass
class Proc:
  name: str
  power: float
  msgs: List[str]
  rtol: float = 0.05
  atol: float = 0.12

  @property
  def range(self):
    tol = max(self.rtol * self.power, self.atol)
    return self.power - tol, self.power + tol

PROCS = [
  Proc('camerad', 2.1, atol=0.2, msgs=['roadCameraState', 'wideRoadCameraState', 'driverCameraState']),
  Proc('modeld', 0.93, atol=0.2, msgs=['modelV2']),
  #Proc('dmonitoringmodeld', 0.4, msgs=['driverStateV2']),
  #Proc('encoderd', 0.23, msgs=['roadEncodeData', 'wideRoadEncodeData', 'driverEncodeData']),
  #Proc('mapsd', 0.05, msgs=['mapRenderState']),
  #Proc('navmodeld', 0.05, msgs=['navModel']),
]

def send_llk_msg(done):
  # Send liveLocationKalman at 20Hz
  pm = messaging.PubMaster(['liveLocationKalman'])
  while not done.is_set():
    msg = gen_llk()
    pm.send('liveLocationKalman', msg)
    time.sleep(1/20.)


class TestPowerDraw(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    if not TICI:
      raise unittest.SkipTest

  def setUp(self):
    HARDWARE.initialize_hardware()
    HARDWARE.set_power_save(False)
    time.sleep(5)

  def tearDown(self):
    manager_cleanup()

  def test_camera_procs(self):
    done = threading.Event()
    thread = threading.Thread(target=send_llk_msg, args=(done,), daemon=True)
    thread.start()

    baseline = get_power(15)
    prev = baseline
    used = {}
    freqs = {}
    for proc in PROCS:
      socks = {msg: messaging.sub_sock(msg) for msg in proc.msgs}
      managed_processes[proc.name].start()
      time.sleep(2.0)
      for sock in socks.values():
        messaging.drain_sock_raw(sock)

      st = time.monotonic()
      now = wait_for_power(proc.range[0] + prev, proc.range[1] + prev, 5, 30)
      dt = time.monotonic() - st
      print(proc.name, f"took {dt:.2f}s", proc.range)
      used[proc.name] = now - prev
      prev = now

      msg_count = sum(len(messaging.drain_sock_raw(sock)) for sock in socks.values())
      freqs[proc.name] = msg_count / dt / len(socks)

    done.set()
    manager_cleanup()

    tab = [['process', 'expected (W)', 'measured (W)', 'expected (Hz)', 'measured (Hz)']]
    for proc in PROCS:
      cur = used[proc.name]
      expected = proc.power
      expected_freq = np.mean([service_list[msg].frequency for msg in proc.msgs])
      tab.append([proc.name, round(expected, 2), round(cur, 2), expected_freq, round(freqs[proc.name], 1)])
      with self.subTest(proc=proc.name):
        self.assertTrue(math.isclose(cur, expected, rel_tol=proc.rtol, abs_tol=proc.atol))
        self.assertTrue(math.isclose(expected_freq, freqs[proc.name], rel_tol=.02, abs_tol=2))
    print()
    print(tabulate(tab, tablefmt="simple_grid"))
    print(f"Baseline {baseline:.2f}W\n")


if __name__ == "__main__":
  unittest.main()
