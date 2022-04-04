from typing import Optional
from .AgentMsg import AgentMsg


class HomePositionMsg(AgentMsg):
    """
    Signal a home position reset.

    The home position can be reset to the current location or set to a specific location.
    Note that if the home location is being reset to the current position, then the location
    is not required. If the home location is being set to a specific location, all components
    must be set.

    ..py:attribute:: latitude
        Latitude

    ..py:attribute:: longitude
        Longitude

    ..py:attribute:: altitude
        Altitude (m)
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

        :param lat: The latitude of the home position
        :type lat: float

        :param lon: The longitude of the home position
        :type lon: float

        :param alt: The altitude of the home position
        :type: alt: float

        :param msg_timeout: The amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            check_ack is set to true.
        :type msg_timeout: float

        :param ack_timeout: The amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when check_ack is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date.
        :type ack_timeout: float

        :param state_timeout: The amount of time that pymavswarm should wait for a
            given agent's state to change after receiving a mavlink message
        :type state_timeout: float

        :param state_delay: The amount of time that pymavswarm should wait after
            sending a command prior to sending another command. This parameter is used
            for sequence-driven commands such as the full takeoff command sequence.
        :type state_delay: float

        :param validate_state: Flag indicating that pymavswarm should check to ensure
            that the message caused the desired state change in the system
        :type validate_state: bool
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
        return self.__altitude

    @property
    def latitude(self) -> float:
        return self.__latitude

    @property
    def longitude(self) -> float:
        return self.__longitude
