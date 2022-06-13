from pymavswarm.messages import AgentMessage


class WaypointMessage(AgentMessage):
    """
    Desired waypoint for an agent to fly to.
    """

    def __init__(
        self,
        lat: float,
        lon: float,
        alt: float,
        hold: float,
        accept_radius: float,
        pass_radius: float,
        yaw: float,
        target_system: int,
        target_comp: int,
        retry: bool,
        msg_timeout: float = 5.0,
        ack_timeout: float = 1.0,
        state_timeout: float = 5.0,
        state_delay: float = 3.0,
        optional_context_props: dict = {},
    ) -> None:
        """
        Constructor.

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

        :param target_system: The target system ID
        :type target_system: int

        :param target_comp: The target component ID
        :type target_comp: int

        :param retry: Indicate whether pymavswarm should retry sending the message
            until acknowledgement
        :type retry: bool

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

        :param optional_context_props: optional properties to append to the message
            context, defaults to {}
        :type optional_context_props: dict, optional
        """
        super().__init__(
            "WAYPOINT",
            target_system,
            target_comp,
            retry,
            msg_timeout=msg_timeout,
            ack_timeout=ack_timeout,
            state_timeout=state_timeout,
            state_delay=state_delay,
            optional_context_props=optional_context_props,
        )
        self.__hold = hold
        self.__accept_radius = accept_radius
        self.__pass_radius = pass_radius
        self.__yaw = yaw
        self.__latitude = lat
        self.__longitude = lon
        self.__altitude = alt

        return

    @property
    def hold(self) -> float:
        """
        Time to stay at waypoint for rotary wing (ignored by fixed wing).

        :rtype: float
        """
        return self.__hold

    @property
    def accept_radius(self) -> float:
        """
        If the sphere with this radius (m) is hit, the waypoint counts as reached.

        :rtype: float
        """
        return self.__accept_radius

    @property
    def pass_radius(self) -> float:
        """
        0 to pass through the WP, if > 0 radius to pass by WP. Positive value for
        clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory
        control.

        :rtype: float
        """
        return self.__pass_radius

    @property
    def yaw(self) -> float:
        """
        Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw
        heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).

        :rtype: float
        """
        return self.__yaw

    @property
    def latitude(self) -> float:
        """
        Latitude of the waypoint.

        :rtype: float
        """
        return self.__latitude

    @property
    def longitude(self) -> float:
        """
        Longitude of the waypoint.

        :rtype: float
        """
        return self.__longitude

    @property
    def altitude(self) -> float:
        """
        Altitude of the waypoint.

        :rtype: float
        """
        return self.__altitude

    @property
    def context(self) -> dict:
        """
        Context of the message.

        :rtype: dict
        """
        context = super().context

        # Update to include new properties
        context["yaw"] = self.__yaw
        context["pass_radius"] = self.__pass_radius
        context["accept_radius"] = self.__accept_radius
        context["hold"] = self.__hold
        context["latitude"] = self.__latitude
        context["longitude"] = self.__longitude
        context["altitude"] = self.__altitude

        return context
