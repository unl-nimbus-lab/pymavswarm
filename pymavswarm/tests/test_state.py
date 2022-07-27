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

from pymavswarm.state import State


class TestState(unittest.TestCase):
    """Test the State class."""

    def test_state_changed_event(self) -> None:
        """Test whether the system properly notifies callbacks on property changes."""
        # Create an example class to test property change signals
        class TestClass(State):
            """Test class."""

            def __init__(self) -> None:
                """Test constructor."""
                super().__init__()

                self.__test_prop = "test"

            @property
            def context(self) -> dict:
                """
                Test context.

                :return: context
                :rtype: dict
                """
                return {"value": self.__test_prop}

            @property
            def test_prop(self) -> str:
                """
                Test property.

                :return: test property
                :rtype: str
                """
                return self.__test_prop

            @test_prop.setter
            def test_prop(self, value: str) -> None:
                """
                Set the test property.

                :param value: value to set the property to
                :type value: str
                """
                self.__test_prop = value
                self.state_changed_event.notify(**self.context)

                return

        test_class = TestClass()
        total_calls = 0
        passed_value = None

        # Sample listener to attach to the event
        def test_fn(value=None) -> None:
            nonlocal total_calls, passed_value
            total_calls += 1
            passed_value = value

            return

        # Connect the listener to the event
        test_class.state_changed_event.add_listener(test_fn)

        # Set the property to a test value
        test_value = "my-test"
        test_class.test_prop = test_value

        # Verify that the listener was called
        self.assertEqual(total_calls, 1)
        self.assertEqual(passed_value, test_value)

        return


if __name__ == "__main__":
    unittest.main()
