import unittest

from pymavswarm.msg import SupportedMsgs as supported_msgs


class TestSupportedMsgs(unittest.TestCase):
    def test_get_command(self) -> None:
        """
        Ensure that we can use the the interface to properly get a supported message
        """
        self.assertEqual(supported_msgs.flight_modes.stabilize, "STABILIZE")

        return


if __name__ == "__main__":
    unittest.main()
