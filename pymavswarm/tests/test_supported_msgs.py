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

import unittest

from pymavswarm.messages import SupportedCommands as supported_msgs


class TestSupportedMsgs(unittest.TestCase):
    def test_get_command(self) -> None:
        """
        Ensure that we can use the the interface to properly get a supported message
        """
        self.assertEqual(supported_msgs.flight_modes.stabilize, "STABILIZE")

        return


if __name__ == "__main__":
    unittest.main()
