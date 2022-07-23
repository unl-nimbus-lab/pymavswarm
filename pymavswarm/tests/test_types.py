from __future__ import annotations

import unittest
from typing import Callable

from pymavswarm.types import AgentID, StateVerifier


class TestTypes(unittest.TestCase):
    """Verify that the system types are properly imported."""

    def test_agent_id_import(self) -> None:
        """Test the AgentID type import."""
        self.assertEqual(AgentID, tuple[int, int])  # type: ignore
        return

    def test_state_verifier(self) -> None:
        """Test the StateVerifier type import."""
        self.assertEqual(StateVerifier, Callable[[AgentID], bool])
        return


if __name__ == "__main__":
    unittest.main()
