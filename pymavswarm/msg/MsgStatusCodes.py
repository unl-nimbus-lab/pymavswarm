from dataclasses import dataclass

@dataclass
class MsgStatusCodes:
    # The message was successfully sent
    SUCCESS = 1

    # The message was not acknowledged
    ACK_FAILURE = 2

    # The state change that the message was attempting to accomplish was not verified
    STATE_VALIDATION_FAILURE = 3