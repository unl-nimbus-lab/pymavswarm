from typing import Optional
from pymavswarm.msg.AgentMsg import AgentMsg


class HomePositionMsg(AgentMsg):
    """
    Signal a home position reset.

    The home position can be reset to the current location or set to a specific location.
    Note that if the home location is being reset to the current position, then the location
    is not required. If the home location is being set to a specific location, all components
    must be set.
    """

    def __init__(
        self,
        msg_type: str,
        target_system: int,
        target_comp: int,
        retry: bool,
        lat: Optional[float] = None,
        lon: Optional[float] = None,
        alt: Optional[float] = None,
        msg_timeout: float = 5.0,
        ack_timeout: float = 1.0,
        state_timeout: float = 5.0,
        state_delay: float = 3.0,
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

        :param lat: The latitude of the home position
        :type lat: Optional[float], optional

        :param lon: The longitude of the home position
        :type lon: Optional[float], optional

        :param alt: The altitude of the home position
        :type: alt: Optional[float], optional

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
        )
        self.__altitude = alt
        self.__latitude = lat
        self.__longitude = lon

        return

    @property
    def altitude(self) -> float:
        """
        Altitude (m)

        :rtype: float
        """
        return self.__altitude

    @property
    def latitude(self) -> float:
        """
        Latitude of the position

        :rtype: float
        """
        return self.__latitude

    @property
    def longitude(self) -> float:
        """
        Longitude of the position

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
