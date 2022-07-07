# pymavswarm is an interface for swarm control and interaction
# Copyright (C) 2022  Evan Palmer

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

# pylint: disable=unused-import

import unittest

from pymavswarm.tests.test_event import TestEvent
from pymavswarm.tests.test_generic import TestGeneric
from pymavswarm.tests.test_mission import TestMission
from pymavswarm.tests.test_notifier_dict import TestNotifierDict
from pymavswarm.tests.test_state import TestState

if __name__ == "__main__":
    unittest.main()