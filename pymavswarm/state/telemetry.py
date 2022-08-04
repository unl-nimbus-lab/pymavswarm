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

from pymavswarm.state.state import State


class Telemetry(State):
    """Telemetry state information."""

    def __init__(
        self, drop_rate: float, optional_context_props: dict | None = None
    ) -> None:
        """
        Create a new telemetry status object.

        :param drop_rate: communication drop rate, (UART, I2C, SPI, CAN), dropped
            packets on all links (packets that were corrupted on reception on the MAV)
        :type drop_rate: float
        :param optional_context_props: optional context properties, defaults to None
        :type optional_context_props: dict | None, optional
        """
        super().__init__(optional_context_props)

        self.__drop_rate = drop_rate

        return

    @property
    def drop_rate(self) -> float:
        """
        Drop rate.

        Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links
        (packets that were corrupted on reception on the MAV)

        :return: drop rate
        :rtype: float
        """
        return self.__drop_rate

    @drop_rate.setter
    def drop_rate(self, rate: float) -> None:
        """
        Set the drop rate.

        :param rate: drop rate [c%]
        :type rate: float
        """
        prev_drop_rate = self.__drop_rate
        self.__drop_rate = rate

        # Signal state change event
        if self.__drop_rate != prev_drop_rate:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def context(self) -> dict:
        """
        Telemetry state context.

        :return: context
        :rtype: dict
        """
        context = super().context
        context["drop_rate"] = self.__drop_rate

        return context

    def __str__(self) -> str:
        """
        Print telemetry information in a human-readable format.

        :return: telemetry information
        :rtype: str
        """
        return f"Telemetry: {self.context}"
