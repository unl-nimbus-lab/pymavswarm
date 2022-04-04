from .AgentMsg import AgentMsg


class WaypointMsg(AgentMsg):
    """
    Desired waypoint for an agent to fly to

    .py:attribute:: hold
        Time to stay at waypoint for rotary wing (ignored by fixed wing)

    .py:attribute:: accept_radius
        If the sphere with this radius (m) is hit, the waypoint counts as reached

    .py:attribute:: pass_radius
        0 to pass through the WP, if > 0 radius to pass by WP. Positive value for
        clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory
        control.

    .py:attribute:: yaw
        Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw
        heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).

    .py:attribute:: latitude
        Latitude of the waypoint

    .py:attribute:: longitude
        Longitude of the waypoint

    .py:attribute:: altitude
        Altitude of the waypoint
    """

    def __init__(
        self,
        hold: float,
        accept_radius: float,
        pass_radius: float,
        yaw: float,
        lat: float,
        lon: float,
        alt: float,
        msg_type: str,
        target_system: int,
        target_comp: int,
        retry: bool,
        msg_timeout: float = 5.0,
        ack_timeout: float = 1.0,
        state_timeout: float = 5.0,
        state_delay: float = 3.0,
        validate_state: bool = False,
    ) -> None:
        """
        :param hold: Time to stay at waypoint for rotary wing (ignored by fixed wing)
        :type hold: float

        :param accept_radius: If the sphere with this radius (m) is hit, the waypoint
            counts as reached
        :type accept_radius: float

        :param pass_radius: 0 to pass through the WP, if > 0 radius to pass by WP.
            Positive value for clockwise orbit, negative value for counter-clockwise
            orbit. Allows trajectory control.
        :type pass_radius: float

        :param yaw: Desired yaw angle at waypoint (rotary wing). NaN to use the current
            system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
        :type yaw: float

        :param lat: Latitude of the waypoint
        :type lat: float

        :param lon: Longitude of the waypoint
        :type lon: float

        :param alt: Altitude of the waypoint
        :type alt: float

        :param msg_type: The sub-message type for a message
        :type msg_type: str

        :param target_system: The target system ID
        :type target_system: int

        :param target_comp: The target component ID
        :type target_comp: int

        :param retry: Indicate whether pymavswarm should retry sending the message
            until acknowledgement
        :type retry: bool

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
        self.__hold = hold
        self.__accept_radius = accept_radius
        self.__pass_radius = pass_radius
        self.__yaw = yaw
        self.__lat = lat
        self.__lon = lon
        self.__alt = alt

        return

    @property
    def hold(self) -> float:
        return self.__hold

    @property
    def accept_radius(self) -> float:
        return self.__accept_radius

    @property
    def pass_radius(self) -> float:
        return self.__pass_radius

    @property
    def yaw(self) -> float:
        return self.__yaw

    @property
    def latitude(self) -> float:
        return self.__lat

    @property
    def longitude(self) -> float:
        return self.__lon

    @property
    def altitude(self) -> float:
        return self.__alt
