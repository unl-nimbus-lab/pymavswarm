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

from __future__ import annotations

from typing import Any, Callable

from pymavswarm import Agent

# AgentID type alias. The provided order should be (system ID, component ID).
AgentID = tuple[int, int]

# A message response code. The provided order should be (code number, code message).
MessageCode = tuple[int, str]

# A method that checks for a state change in the target agent after a message is sent.
StateVerifier = Callable[[AgentID], bool]

# A method that sends the desired message.
CommandExecutor = Callable[[AgentID], None]


# A method that performs some behavior after sending a message.
PostExecutionHandler = Callable[
    [
        AgentID,
        bool,
        tuple[int, str],
        dict | None,
    ],
    None,
]

# A callback method responsible for processing an incoming MAVLink message.
MessageHandler = Callable[[Any, dict[AgentID, Agent]], None]
