import sys

sys.path.append("..")

import unittest
from Parameter import Parameter


class TestParameter(unittest.TestCase):
    def test_invalid_timeout_init(self) -> None:
        """
        Test invalid timeout
        """
        self.assertRaises(ValueError, Parameter, 0, 0, "test", False, None, -1.0)

        return

    def test_parameter_read_result_event(self) -> None:
        """
        Test the parameter_read_result_event to evaluate whether the event properly
        notifies handlers
        """
        # Create a new message
        param = Parameter(0, 0, "test", False)

        total_calls = 0
        passed_result = None
        passed_context = None

        # Sample listener to attach to the event
        def test_fn(kwargs) -> None:
            nonlocal total_calls, passed_result, passed_context
            total_calls += 1
            passed_result = kwargs["result"]
            passed_context = kwargs["context"]

            return

        # Register the listener
        param.parameter_read_result_event.add_listener(test_fn)

        # Test values
        result = False

        # Signal the event
        param.parameter_read_result_event.notify(result=result, context=param.context)

        # Verify that all properties were passed successfully
        self.assertEqual(total_calls, 1)
        self.assertEqual(passed_result, result)
        self.assertEqual(passed_context, param.context)

        return

    def test_parameter_write_result_event(self) -> None:
        """
        Test the parameter_write_result_event to evaluate whether the event properly
        notifies handlers
        """
        # Create a new message
        param = Parameter(0, 0, "test", False)

        total_calls = 0
        passed_result = None
        passed_context = None

        # Sample listener to attach to the event
        def test_fn(kwargs) -> None:
            nonlocal total_calls, passed_result, passed_context
            total_calls += 1
            passed_result = kwargs["result"]
            passed_context = kwargs["context"]

            return

        # Register the listener
        param.parameter_write_result_event.add_listener(test_fn)

        # Test values
        result = False

        # Signal the event
        param.parameter_write_result_event.notify(result=result, context=param.context)

        # Verify that all properties were passed successfully
        self.assertEqual(total_calls, 1)
        self.assertEqual(passed_result, result)
        self.assertEqual(passed_context, param.context)

        return


if __name__ == "__main__":
    unittest.main()
