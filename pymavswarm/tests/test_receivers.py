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
from typing import Any

from pymavswarm import Agent
from pymavswarm.handlers.receivers import Receivers
from pymavswarm.types import AgentID


class TestReceivers(unittest.TestCase):
    """Test the Receivers class."""

    def test_add_single_message_handler(self) -> None:
        """Verify that one method is properly added to the handlers."""
        receivers = Receivers()

        def test_func(message: Any, agents: dict[AgentID, Agent]):
            pass

        test_message_type = "test"

        receivers.add_message_handler(test_message_type, test_func)

        self.assertIn(test_func, receivers.receivers[test_message_type])

        return

    def test_add_multiple_message_handlers(self) -> None:
        """Verify that multiple methods are properly added to the handlers."""
        receivers = Receivers()

        def test_func(message: Any, agents: dict[AgentID, Agent]):
            pass

        def test_func_two(message: Any, agents: dict[AgentID, Agent]):
            pass

        test_message_type = "test"

        receivers.add_message_handler(test_message_type, test_func)
        receivers.add_message_handler(test_message_type, test_func_two)

        self.assertIn(test_func, receivers.receivers[test_message_type])
        self.assertIn(test_func_two, receivers.receivers[test_message_type])

        return


if __name__ == "__main__":
    unittest.main()
