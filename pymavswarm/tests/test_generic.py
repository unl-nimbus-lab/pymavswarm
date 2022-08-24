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

from pymavswarm.state import Generic


class TestGeneric(unittest.TestCase):
    """Test the Generic class."""

    def test_state_changed_event(self) -> None:
        """Test whether the system properly notifies callbacks on property changes."""
        test_prop_name = "test"

        # Create an example class to test property change signals
        class TestClass:
            """Test class."""

            def __init__(self) -> None:
                """Test constructor."""
                super().__init__()

                self.__test_prop = Generic(test_prop_name, "random")

                return

            @property
            def test_prop(self) -> Generic:
                """
                Test property.

                :return: test property
                :rtype: str
                """
                return self.__test_prop

        test_class = TestClass()
        total_calls = 0
        passed_value = None

        # Sample listener to attach to the event
        def test_fn(test=None) -> None:
            nonlocal total_calls, passed_value
            total_calls += 1
            passed_value = test

            return

        # Connect the listener to the event
        test_class.test_prop.state_changed_event.add_listener(test_fn)

        # Set the property to a test value
        test_value = "my-test"
        test_class.test_prop.value = test_value

        # Verify that the listener was called
        self.assertEqual(total_calls, 1)
        self.assertEqual(passed_value, test_value)

        return


if __name__ == "__main__":
    unittest.main()
