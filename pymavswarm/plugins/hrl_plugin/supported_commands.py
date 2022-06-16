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

"""Supported HRL commands."""

from typing import List


class HRLCommands:
    """Supported HRL commands."""

    start_path_execution = 0
    reset_path_execution = 1
    stop_path_execution = 2
    start_live_execution = 3

    @staticmethod
    def get_supported_types() -> List[int]:
        """
        Get the supported HRL commands.

        :return: supported HRL commands
        :rtype: List[int]
        """
        return [
            HRLCommands.start_path_execution,
            HRLCommands.reset_path_execution,
            HRLCommands.stop_path_execution,
            HRLCommands.start_live_execution,
        ]
