import sys

sys.path.append("..")

import unittest
from pymavswarm.event.event import Event


class TestEvent(unittest.TestCase):
    def test_add_listener(self) -> None:
        """
        Test adding a listener function to the event listeners
        """
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
        """
        Test removing an existing listener function
        """
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
        """
        Test notifying a function without arguments
        """
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
        """
        Test notifying a function with arguments
        """
        # Create a new event
        event = Event()

        # Variable to manipulate with the test function
        counter = 0

        # Function used to increment the counter by one
        def test_fn(kwargs):
            nonlocal counter
            counter += kwargs["addend"]

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
