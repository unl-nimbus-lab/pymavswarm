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

from pymavswarm.utils import Event, NotifierDict


class TestNotifierDict(unittest.TestCase):
    """Test the NotifierDict class."""

    def test_add_element(self) -> None:
        """Ensure that notifiers are called when an item is added to the dict."""
        # Create a new event
        event = Event()

        # Specify the desired key, value pair
        test_key = "test_key"
        test_value = "test_value"

        # Reset these with the test function
        result_key = None
        result_value = None
        operation = None

        # Function used to modify the resulting key and value
        def test_fn(kwargs):
            nonlocal result_key, result_value, operation
            operation = kwargs["operation"]
            result_key = kwargs["key"]
            result_value = kwargs["value"]

        # Register the function as a listener
        event.add_listener(test_fn)

        # Create a notifier dict with the test event
        test_dict = NotifierDict(event)

        # Set the value of the test key
        test_dict[test_key] = test_value

        self.assertEqual(operation, "set")
        self.assertEqual(test_key, result_key)
        self.assertEqual(test_value, result_value)

    def test_delete_element(self) -> None:
        """Ensure that notifiers are called when an item is deleted from the dict."""
        # Create a new event
        event = Event()

        # Specify the desired key, value pair
        test_key = "test_key"
        test_value = "test_value"

        # Reset these with the test function
        result_key = None
        result_value = None
        operation = None

        # Function used to modify the resulting key and value
        def test_fn(kwargs):
            nonlocal result_key, result_value, operation
            operation = kwargs["operation"]
            result_key = kwargs["key"]
            result_value = kwargs["value"]

        # Register the function as a listener
        event.add_listener(test_fn)

        # Create a notifier dict with the test event
        test_dict = NotifierDict(event)

        # Set the value of the test key
        test_dict[test_key] = test_value

        # Delete the test value
        del test_dict[test_key]

        self.assertEqual(operation, "del")
        self.assertEqual(test_key, result_key)
        self.assertEqual(test_value, result_value)


if __name__ == "__main__":
    unittest.main()
