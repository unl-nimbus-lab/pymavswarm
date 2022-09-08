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

from pymavswarm.mission.stage import Stage
from pymavswarm.types import AgentID


class Mission:
    def __init__(self, stages: list[Stage] | None = None) -> None:
        if stages is None:
            self.__stages: list[Stage] = []
        else:
            self.__stages = stages

        return

    @property
    def stages(self) -> list[Stage]:
        """
        Sequence of commands executed together within a mission.

        Stages can be strung together to create complex missions.

        :return: _description_
        :rtype: list[Stage]
        """
        return self.__stages

    @property
    def target_agent_ids(self) -> list[AgentID]:
        agent_ids: list[AgentID] = []

        for stage in self.__stages:
            agent_ids.extend(stage.target_agent_ids)

        return [*set(agent_ids)]
