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

from typing import List, Optional

from pymavswarm.handlers import MessageSenders
from pymavswarm.messages.agent_message import AgentMessage


class PreflightCalibrationCommand(AgentMessage):
    """Perform pre-flight calibration on an agent."""

    GYROSCOPE_CALIBRATION = MessageSenders.GYROSCOPE_CALIBRATION
    MAGNETOMETER_CALIBRATION = MessageSenders.MAGNETOMETER_CALIBRATION
    GROUND_PRESSURE_CALIBRATION = MessageSenders.GROUND_PRESSURE_CALIBRATION
    AIRSPEED_CALIBRATION = MessageSenders.AIRSPEED_CALIBRATION
    BAROMETER_TEMPERATURE_CALIBRATION = MessageSenders.BAROMETER_TEMPERATURE_CALIBRATION
    ACCELEROMETER_CALIBRATION = MessageSenders.ACCELEROMETER_CALIBRATION
    SIMPLE_ACCELEROMETER_CALIBRATION = MessageSenders.SIMPLE_ACCELEROMETER_CALIBRATION
    AHRS_TRIM = MessageSenders.AHRS_TRIM

    @classmethod
    def get_calibration_types(cls) -> List[str]:
        """
        Get the supported pre-flight calibration types.

        :return: pre-flight calibration types
        :rtype: List[str]
        """
        return [
            cls.GYROSCOPE_CALIBRATION,
            cls.MAGNETOMETER_CALIBRATION,
            cls.GROUND_PRESSURE_CALIBRATION,
            cls.AIRSPEED_CALIBRATION,
            cls.BAROMETER_TEMPERATURE_CALIBRATION,
            cls.ACCELEROMETER_CALIBRATION,
            cls.SIMPLE_ACCELEROMETER_CALIBRATION,
            cls.AHRS_TRIM,
        ]

    def __init__(
        self,
        calibration_type: str,
        target_system: int,
        target_component: int,
        retry: bool,
        message_timeout: float = 5,
        ack_timeout: float = 1,
        state_timeout: float = 5,
        state_delay: float = 3,
        optional_context_props: Optional[dict] = None,
    ) -> None:
        """
        Create a pre-flight calibration command.

        :param calibration_type: type of pre-flight calibration to perform
        :type calibration_type: str

        :param target_system: target system ID
        :type target_system: int

        :param target_component: target component ID
        :type target_component: int

        :param retry: indicate whether pymavswarm should retry sending the message
            until acknowledgement
        :type retry: bool

        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 5.0
        :type message_timeout: float, optional

        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 1.0
        :type ack_timeout: float, optional

        :param state_timeout: amount of time that pymavswarm should wait for a
            given agent's state to change after receiving a mavlink message, defaults
            to 5.0
        :type state_timeout: float, optional

        :param state_delay: amount of time that pymavswarm should wait after
            sending a command prior to sending another command. This parameter is used
            for sequence-driven commands such as the full takeoff command sequence,
            defaults to 3.0
        :type state_delay: float, optional

        :param optional_context_props: optional properties to append to the message
            context, defaults to None
        :type optional_context_props: Optional[dict], optional

        :raises ValueError: invalid pre-flight calibration type
        """
        if calibration_type not in PreflightCalibrationCommand.get_calibration_types():
            raise ValueError(
                f"{calibration_type} is not a supported pre-flight calibration type. "
                "Supported pre-flight calibration types include: "
                f"{PreflightCalibrationCommand.get_calibration_types()}"
            )

        super().__init__(
            calibration_type,
            target_system,
            target_component,
            retry,
            message_timeout,
            ack_timeout,
            state_timeout,
            state_delay,
            optional_context_props,
        )

        self.__calibration_type = calibration_type

        return

    @property
    def calibration_type(self) -> str:
        """
        Type of calibration to perform.

        :return: pre-flight calibration type
        :rtype: str
        """
        return self.__calibration_type

    @property
    def context(self) -> dict:
        """
        Message context.

        :return: current message context
        :rtype: dict
        """
        context = super().context
        context["calibration_type"] = self.__calibration_type

        return context
