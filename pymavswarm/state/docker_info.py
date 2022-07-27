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

import datetime

from pymavswarm.state.state import State


class DockerInfo(State):
    """Information related to the agent's deployed Docker image."""

    def __init__(
        self,
        version: str,
        last_update: datetime.datetime,
        optional_context_props: dict | None = None,
    ) -> None:
        """
        Create a new docker info instance.

        :param version: version of the Docker image deployed on an agent
        :type version: str
        :param last_update: date that the Docker image deployed on the agent
            was last updated
        :type last_update: datetime.datetime
        :param optional_context_props: optional properties to add to the DockerInfo
            context, defaults to None
        :type optional_context_props: dict | None, optional
        """
        super().__init__(optional_context_props)

        self.__version = version
        self.__last_update = last_update

        return

    @property
    def version(self) -> str:
        """
        Docker image version deployed on an agent.

        :return: docker image version
        :rtype: str
        """
        return self.__version

    @version.setter
    def version(self, version: str) -> None:
        """
        Set the Docker image version.

        :param version: image version
        :type version: str
        """
        prev_version = self.__version
        self.__version = version

        # Signal state change event
        if self.__version != prev_version:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def last_update(self) -> datetime.datetime:
        """
        Date that the image was last updated.

        :return: last update date
        :rtype: str
        """
        return self.__last_update

    @last_update.setter
    def last_update(self, date: datetime.datetime) -> None:
        """
        Set the date that the image was last updated.

        :param date: date that the agent's image was last updated
        :type date: str
        """
        prev_last_update = self.__last_update
        self.__last_update = date

        # Signal state change event
        if self.__last_update != prev_last_update:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def context(self) -> dict:
        """
        Docker info context.

        :return: context
        :rtype: dict
        """
        context = super().context

        context["version"] = self.__version
        context["last_update"] = self.__last_update

        return context

    def __str__(self) -> str:
        """
        Print docker information in a human-readable format.

        :return: docker information
        :rtype: str
        """
        return f"DockerInfo: {self.context}"
