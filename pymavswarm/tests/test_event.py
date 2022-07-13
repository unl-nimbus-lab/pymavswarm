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

from pymavswarm.utils import Event


class TestEvent(unittest.TestCase):
    """Test the Event class."""

    def test_add_listener(self) -> None:
        """Test adding a listener function to the event listeners."""
        # Create a new event
        event = Event()

        # Test function to add as a listener
        def test_fn():
            pass

        # Add the new listener
        event.add_listener(test_fn)

        # Ensure that the listener was properly registered
        self.assertIn(test_fn, event.listeners)

        return

    def test_remove_listener(self) -> None:
        """Test removing an existing listener function."""
        # Create a new event
        event = Event()

        # Test function to add as a listener
        def test_fn():
            pass

        # Add the new listener
        event.add_listener(test_fn)

        # Now, remove the listener
        event.remove_listener(test_fn)

        self.assertNotIn(test_fn, event.listeners)

        return

    def test_notify_no_args(self) -> None:
        """Test notifying a function without arguments."""
        # Create a new event
        event = Event()

        # The number of calls made to the test_fn method
        calls = 0

        # Function used to increment the counter by one
        def test_fn() -> None:
            nonlocal calls
            calls += 1

            return

        # Register the function as a listener
        event.add_listener(test_fn)

        # Signal the event
        event.notify()

        # Ensure that the counter was incremented by one
        self.assertEqual(calls, 1)

        return

    def test_notify_multiple_args(self) -> None:
        """Test notifying a function with arguments."""
        # Create a new event
        event = Event()

        # Variable to manipulate with the test function
        counter = 0

        # Function used to increment the counter by one
        def test_fn(addend=0):
            nonlocal counter
            counter += addend

        # Register the function as a listener
        event.add_listener(test_fn)

        increment = 5

        # Signal the event
        event.notify(addend=increment)

        # Ensure that the counter was incremented by one
        self.assertEqual(counter, increment)

        return


if __name__ == "__main__":
    unittest.main()
