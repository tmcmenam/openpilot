#!/usr/bin/env python3
import argparse
import os
import pytest
import sys
import unittest

from parameterized import parameterized, parameterized_class

from openpilot.selfdrive.car.car_helpers import interface_names
from openpilot.selfdrive.test.openpilotci import get_url, upload_file
from openpilot.selfdrive.test.process_replay.compare_logs import compare_logs
from openpilot.selfdrive.test.process_replay.process_replay import CONFIGS, PROC_REPLAY_DIR, FAKEDATA, check_openpilot_enabled, replay_process
from openpilot.system.version import get_commit
from openpilot.tools.lib.filereader import FileReader
from openpilot.tools.lib.helpers import save_log
from openpilot.tools.lib.logreader import LogReader


source_segments = [
  ("BODY", "937ccb7243511b65|2022-05-24--16-03-09--1"),        # COMMA.BODY
  ("HYUNDAI", "02c45f73a2e5c6e9|2021-01-01--19-08-22--1"),     # HYUNDAI.SONATA
  ("HYUNDAI2", "d545129f3ca90f28|2022-11-07--20-43-08--3"),    # HYUNDAI.KIA_EV6 (+ QCOM GPS)
  ("TOYOTA", "0982d79ebb0de295|2021-01-04--17-13-21--13"),     # TOYOTA.PRIUS
  ("TOYOTA2", "0982d79ebb0de295|2021-01-03--20-03-36--6"),     # TOYOTA.RAV4
  ("TOYOTA3", "f7d7e3538cda1a2a|2021-08-16--08-55-34--6"),     # TOYOTA.COROLLA_TSS2
  ("HONDA", "eb140f119469d9ab|2021-06-12--10-46-24--27"),      # HONDA.CIVIC (NIDEC)
  ("HONDA2", "7d2244f34d1bbcda|2021-06-25--12-25-37--26"),     # HONDA.ACCORD (BOSCH)
  ("CHRYSLER", "4deb27de11bee626|2021-02-20--11-28-55--8"),    # CHRYSLER.PACIFICA_2018_HYBRID
  ("RAM", "17fc16d840fe9d21|2023-04-26--13-28-44--5"),         # CHRYSLER.RAM_1500
  ("SUBARU", "341dccd5359e3c97|2022-09-12--10-35-33--3"),      # SUBARU.OUTBACK
  ("GM", "0c58b6a25109da2b|2021-02-23--16-35-50--11"),         # GM.VOLT
  ("GM2", "376bf99325883932|2022-10-27--13-41-22--1"),         # GM.BOLT_EUV
  ("NISSAN", "35336926920f3571|2021-02-12--18-38-48--46"),     # NISSAN.XTRAIL
  ("VOLKSWAGEN", "de9592456ad7d144|2021-06-29--11-00-15--6"),  # VOLKSWAGEN.GOLF
  ("MAZDA", "bd6a637565e91581|2021-10-30--15-14-53--4"),       # MAZDA.CX9_2021
  ("FORD", "54827bf84c38b14f|2023-01-26--21-59-07--4"),        # FORD.BRONCO_SPORT_MK1

  # Enable when port is tested and dashcamOnly is no longer set
  #("TESLA", "bb50caf5f0945ab1|2021-06-19--17-20-18--3"),      # TESLA.AP2_MODELS
  #("VOLKSWAGEN2", "3cfdec54aa035f3f|2022-07-19--23-45-10--2"),  # VOLKSWAGEN.PASSAT_NMS
]

segments = [
  ("BODY", "aregenECF15D9E559|2023-05-10--14-26-40--0"),
  ("HYUNDAI", "aregenAB9F543F70A|2023-05-10--14-28-25--0"),
  ("HYUNDAI2", "aregen39F5A028F96|2023-05-10--14-31-00--0"),
  ("TOYOTA", "aregen8D6A8B36E8D|2023-05-10--14-32-38--0"),
  ("TOYOTA2", "aregenB1933C49809|2023-05-10--14-34-14--0"),
  ("TOYOTA3", "aregen5D9915223DC|2023-05-10--14-36-43--0"),
  ("HONDA", "aregen484B732B675|2023-05-10--14-38-23--0"),
  ("HONDA2", "aregenAF6ACED4713|2023-05-10--14-40-01--0"),
  ("CHRYSLER", "aregen99B094E1E2E|2023-05-10--14-41-40--0"),
  ("RAM", "aregen5C2487E1EEB|2023-05-10--14-44-09--0"),
  ("SUBARU", "aregen98D277B792E|2023-05-10--14-46-46--0"),
  ("GM", "aregen377BA28D848|2023-05-10--14-48-28--0"),
  ("GM2", "aregen7CA0CC0F0C2|2023-05-10--14-51-00--0"),
  ("NISSAN", "aregen7097BF01563|2023-05-10--14-52-43--0"),
  ("VOLKSWAGEN", "aregen765AF3D2CB5|2023-05-10--14-54-23--0"),
  ("MAZDA", "aregen3053762FF2E|2023-05-10--14-56-53--0"),
  ("FORD", "aregenDDE0F89FA1E|2023-05-10--14-59-26--0"),
]


# dashcamOnly makes don't need to be tested until a full port is done
excluded_interfaces = ["mock", "tesla"]

BASE_URL = "https://commadataci.blob.core.windows.net/openpilotci/"
REF_COMMIT_FN = os.path.join(PROC_REPLAY_DIR, "ref_commit")
EXCLUDED_PROCS = {"modeld", "dmonitoringmodeld"}


def get_log_data(segment):
  r, n = segment.rsplit("--", 1)
  with FileReader(get_url(r, n)) as f:
    return f.read()


ALL_PROCS = {cfg.proc_name for cfg in CONFIGS if cfg.proc_name not in EXCLUDED_PROCS}
ALL_CARS = {car for car, _ in segments}

CAR_TO_SEGMENT = dict(segments)
PROC_TO_CFG = {cfg.proc_name: cfg for cfg in CONFIGS}

cpu_count = os.cpu_count() or 1


def get_test_parameters():
  parser = argparse.ArgumentParser(description="Regression test to identify changes in a process's output")
  parser.add_argument("--whitelist-procs", type=str, nargs="*", default=ALL_PROCS,
                      help="Whitelist given processes from the test (e.g. controlsd)")
  parser.add_argument("--whitelist-cars", type=str, nargs="*", default=ALL_CARS,
                      help="Whitelist given cars from the test (e.g. HONDA)")
  parser.add_argument("--blacklist-procs", type=str, nargs="*", default=[],
                      help="Blacklist given processes from the test (e.g. controlsd)")
  parser.add_argument("--blacklist-cars", type=str, nargs="*", default=[],
                      help="Blacklist given cars from the test (e.g. HONDA)")
  parser.add_argument("--ignore-fields", type=str, nargs="*", default=[],
                      help="Extra fields or msgs to ignore (e.g. carState.events)")
  parser.add_argument("--ignore-msgs", type=str, nargs="*", default=[],
                      help="Msgs to ignore (e.g. carEvents)")
  parser.add_argument("--update-refs", action="store_true",
                      help="Updates reference logs using current commit")
  parser.add_argument("--upload-only", action="store_true",
                      help="Skips testing processes and uploads logs from previous test run")

  args, left = parser.parse_known_args()
  sys.argv = sys.argv[:1]+left

  tested_procs = set(args.whitelist_procs) - set(args.blacklist_procs)
  tested_cars = set(args.whitelist_cars) - set(args.blacklist_cars)
  tested_cars = {c.upper() for c in tested_cars}

  return tested_cars, tested_procs, args.ignore_fields, args.ignore_msgs, args.update_refs, args.upload_only


TESTED_CARS, TESTED_PROCS, ignore_fields, ignore_msgs, update_refs, upload_only = get_test_parameters()


@pytest.mark.slow
@parameterized_class(('car'), [(c,) for c in sorted(TESTED_CARS)])
class TestProcesses(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    os.makedirs(os.path.dirname(FAKEDATA), exist_ok=True)

    cls.upload = update_refs or upload_only

    try:
      with open(REF_COMMIT_FN) as f:
        cls.ref_commit = f.read().strip()
    except FileNotFoundError:
      print("Couldn't find reference commit")
      sys.exit(1)

    cls.cur_commit = get_commit()
    cls.assertNotEqual(cls.cur_commit, None, "Couldn't get current commit")

    cls.segment = CAR_TO_SEGMENT[cls.car]
    cls.log_bytes = get_log_data(CAR_TO_SEGMENT[cls.car])

  def test_all_makes_are_tested(self):
    # check to make sure all car brands are tested
    untested = (set(interface_names) - set(excluded_interfaces)) - {c.lower() for c in TESTED_CARS}
    self.assertEqual(len(untested), 0, f"Cars missing routes: {str(untested)}")

  def _run_replay(self, cfg):
    lr = LogReader.from_bytes(self.log_bytes)

    try:
      return replay_process(cfg, lr, disable_progress=True)
    except Exception as e:
      raise Exception(f"failed on segment: {self.segment} \n{e}") from e

  @parameterized.expand(sorted(TESTED_PROCS))
  def test_process(self, proc):
    cfg = PROC_TO_CFG[proc]

    cur_log_fn = os.path.join(FAKEDATA, f"{self.segment}_{proc}_{self.cur_commit}.bz2")
    if update_refs:  # reference logs will not exist if routes were just regenerated
      ref_log_path = get_url(*self.segment.rsplit("--", 1))
    else:
      ref_log_fn = os.path.join(FAKEDATA, f"{self.segment}_{proc}_{self.ref_commit}.bz2")
      ref_log_path = ref_log_fn if os.path.exists(ref_log_fn) else BASE_URL + os.path.basename(ref_log_fn)

    if update_refs or upload_only:
      self.assertTrue(os.path.exists(cur_log_fn), f"Cannot find log to upload: {cur_log_fn}")
      upload_file(cur_log_fn, os.path.basename(cur_log_fn))
      os.remove(cur_log_fn)

    if upload_only:
      raise unittest.SkipTest("skipping test, uploading only")

    log_msgs = self._run_replay(cfg)
    save_log(cur_log_fn, log_msgs)

    ref_log_msgs = list(LogReader(ref_log_path))

    diff = compare_logs(ref_log_msgs, log_msgs, ignore_fields + cfg.ignore, ignore_msgs)

    self.assertEqual(diff, [], "Diff not empty")

    if proc == "controlsd":
      with self.subTest("controlsd-enabled"):
        # check to make sure openpilot is engaged in the route
        self.assertTrue(check_openpilot_enabled(log_msgs), f"Route did not enable at all or for long enough: {self.segment}")


if __name__ == "__main__":
  unittest.main()