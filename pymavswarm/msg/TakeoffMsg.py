from typing import Optional
from AgentMsg import AgentMsg


class TakeoffMsg(AgentMsg):
    """
    Takeoff to a certain location/altitude
    """

    def __init__(
        self,
        msg_type: str,
        target_system: int,
        target_comp: int,
        retry: bool,
        alt: float = 3.0,
        lat: Optional[float] = None,
        lon: Optional[float] = None,
        msg_timeout: float = 5.0,
        ack_timeout: float = 1.0,
        state_timeout: float = 5.0,
        state_delay: float = 3.0,
        validate_state: bool = False,
    ) -> None:
        """
        :param msg_type: The sub-message type for a message
        :type msg_type: str

        :param target_system: The target system ID
        :type target_system: int

        :param target_comp: The target component ID
        :type target_comp: int

        :param retry: Indicate whether pymavswarm should retry sending the message
            until acknowledgement
        :type retry: bool

        :param alt: The desired takeoff altitude
        :type alt: float

        :param lat: The desired takeoff latitude (optional)
        :type lat: Optional[float], optional

        :param lon: The desired takeoff longitude (optional)
        :type lon: Optional[float], optional

        :param msg_timeout: The amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 5.0
        :type msg_timeout: float, optional

        :param ack_timeout: The amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 1.0
        :type ack_timeout: float, optional

        :param state_timeout: The amount of time that pymavswarm should wait for a
            given agent's state to change after receiving a mavlink message, defaults
            to 5.0
        :type state_timeout: float, optional

        :param state_delay: The amount of time that pymavswarm should wait after
            sending a command prior to sending another command. This parameter is used
            for sequence-driven commands such as the full takeoff command sequence,
            defaults to 3.0
        :type state_delay: float, optional

        :param validate_state: Flag indicating that pymavswarm should check to ensure
            that the message caused the desired state change in the system, defaults to
            False
        :type validate_state: bool, optional
        """
        super().__init__(
            msg_type,
            target_system,
            target_comp,
            retry,
            msg_timeout=msg_timeout,
            ack_timeout=ack_timeout,
            state_timeout=state_timeout,
            state_delay=state_delay,
            validate_state=validate_state,
        )
        self.__altitude = alt
        self.__latitude = lat
        self.__longitude = lon

        return

    @property
    def altitude(self) -> float:
        """
        The altitude that the agent should takeoff to

        :rtype: float
        """
        return self.__altitude

    @property
    def latitude(self) -> float:
        """
        The latitude of the takeoff waypoint

        :rtype: float
        """
        return self.__latitude

    @property
    def longitude(self) -> float:
        """
        The longitude of the takeoff waypoint

        :rtype: float
        """
        return self.__longitude

    @property
    def context(self) -> dict:
        """
        Get the context of the message

        :rtype: dict
        """
        context = super().context

        # Update to include new properties
        context["latitude"] = self.__latitude
        context["longitude"] = self.__longitude
        context["altitude"] = self.__altitude

        return context
