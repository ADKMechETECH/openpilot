"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Registered test console vehicle profiles (DEV-TEST BUILD ONLY). select_profile() (test_console/profile.py) picks the
first whose applies(CP) is true. Adding a car = a new profile package here + one entry in PROFILES; the core is reused
as is (see docs/TEST_CONSOLE.md, "Adding a car").
"""
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt import PROFILE as CIVIC_6MT

PROFILES = (CIVIC_6MT,)
