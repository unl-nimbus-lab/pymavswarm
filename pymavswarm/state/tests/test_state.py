import sys

sys.path.append("..")

import unittest
from State import State


class TestState(unittest.TestCase):
    def test_state_changed_event(self) -> None:
        """
        Test whether the system properly notifies callbacks on property changes
        """
        # Create an example class to test property change signals
        class TestClass(State):
            def __init__(self) -> None:
                super().__init__()

                self.__test_prop = "test"

            @property
            def context(self) -> dict:
                return {"value": self.__test_prop}

            @property
            def test_prop(self) -> str:
                return self.__test_prop

            @test_prop.setter
            def test_prop(self, value: str) -> None:
                self.__test_prop = value
                self.state_changed_event.notify(context=self.context)
                return

        test_class = TestClass()
        total_calls = 0
        passed_context = None

        # Sample listener to attach to the event
        def test_fn(kwargs) -> None:
            nonlocal total_calls, passed_context
            total_calls += 1
            passed_context = kwargs["context"]

            return

        # Connect the listener to the event
        test_class.state_changed_event.add_listener(test_fn)

        # Set the property to a test value
        test_value = "my-test"
        test_class.test_prop = test_value

        # Verify that the listener was called
        self.assertEqual(total_calls, 1)
        self.assertEqual(passed_context["value"], test_value)

        return


if __name__ == "__main__":
    unittest.main()
