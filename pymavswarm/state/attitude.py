from pymavswarm.state.state import State


class Attitude(State):
    """
    Attitude of an agent
    """

    def __init__(
        self,
        pitch: float = 0.0,
        yaw: float = 0.0,
        roll: float = 0.0,
        pitch_speed: float = 0.0,
        yaw_speed: float = 0.0,
        roll_speed: float = 0.0,
        optional_context_props: dict = {},
    ) -> None:
        """
        :param pitch: Pitch angle (-pi..+pi), defaults to 0.0
        :type pitch: float, optional

        :param yaw: Yaw angle (-pi..+pi), defaults to 0.0
        :type yaw: float, optional

        :param roll: Roll angle (-pi..+pi), defaults to 0.0
        :type roll: float, optional

        :param pitch_speed: Pitch angular speed (rad/s), defaults to 0.0
        :type pitch_speed: float, optional

        :param yaw_speed: Yaw angular speed (rad/s), defaults to 0.0
        :type yaw_speed: float, optional

        :param roll_speed: Roll angular speed (rad/s), defaults to [], defaults to 0.0
        :type roll_speed: float, optional

        :param optional_context_props: Optional properties to add to the context
        :type optional_context_props: dict, optional
        """
        super().__init__()

        self.__pitch = pitch
        self.__yaw = yaw
        self.__roll = roll
        self.__pitch_speed = pitch_speed
        self.__yaw_speed = yaw_speed
        self.__roll_speed = roll_speed
        self.__optional_context_props = optional_context_props

        return

    @property
    def context(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Properties of interest associated with the attitude
        :rtype: dict
        """
        context = {
            "roll": self.__roll,
            "pitch": self.__pitch,
            "yaw": self.__yaw,
            "roll_speed": self.__roll_speed,
            "pitch_speed": self.__pitch_speed,
            "yaw_speed": self.__yaw_speed,
        }
        context.update(self.__optional_context_props)

        return context

    @property
    def pitch(self) -> float:
        """
        Pitch angle (-pi..+pi)

        :rtype: float
        """
        return self.__pitch

    @pitch.setter
    def pitch(self, angle: float) -> None:
        """
        pitch setter

        :param angle: Angle in rad
        :type angle: float
        """
        prev_pitch = self.__pitch
        self.__pitch = angle

        # Signal state change event
        if self.__pitch != prev_pitch:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def yaw(self) -> float:
        """
        Yaw angle (-pi..+pi)

        :rtype: float
        """
        return self.__yaw

    @yaw.setter
    def yaw(self, angle: float) -> None:
        """
        yaw setter

        :param angle: Angle in rad
        :type angle: float
        """
        prev_yaw = self.__yaw
        self.__yaw = angle

        # Signal state change event
        if self.__yaw != prev_yaw:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def roll(self) -> float:
        """
        Roll angle (-pi..+pi)

        :rtype: float
        """
        return self.__roll

    @roll.setter
    def roll(self, angle: float) -> None:
        """
        roll setter

        :param angle: Angle in rad
        :type angle: float
        """
        prev_roll = self.__roll
        self.__roll = angle

        # Signal state change event
        if self.__roll != prev_roll:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def pitch_speed(self) -> float:
        """
        Pitch angular speed (rad/s)

        :rtype: float
        """
        return self.__pitch_speed

    @pitch_speed.setter
    def pitch_speed(self, rate: float) -> None:
        """
        pitch_speed setter

        :param rate: Rate in rad/s
        :type rate: float
        """
        prev_pitch_speed = self.__pitch_speed
        self.__pitch_speed = rate

        # Signal state change event
        if self.__pitch_speed != prev_pitch_speed:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def roll_speed(self) -> float:
        """
        Roll angular speed (rad/s)

        :rtype: float
        """
        return self.__roll_speed

    @roll_speed.setter
    def roll_speed(self, rate: float) -> None:
        """
        roll_speed setter

        :param rate: Rate in rad/s
        :type rate: float
        """
        prev_roll_speed = self.__roll_speed
        self.__roll_speed = rate

        # Signal state change event
        if self.__roll_speed != prev_roll_speed:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def yaw_speed(self) -> float:
        """
        Yaw angular speed (rad/s)

        :rtype: float
        """
        return self.__yaw_speed

    @yaw_speed.setter
    def yaw_speed(self, rate: float) -> None:
        """
        yaw_speed setter

        :param rate: Rate in rad/s
        :type rate: float
        """
        prev_yaw_speed = self.__yaw_speed
        self.__yaw_speed = rate

        # Signal state change event
        if self.__yaw_speed != prev_yaw_speed:
            self.state_changed_event.notify(context=self.context)

        return
