# def receive_message(msg):
#     def decorator(f):
#         def wrapper(self, *args, **kwargs):
#             f(self, *args, **kwargs)


# def receive_message(self, msg: Union[list, str]) -> Callable:
#     def decorator(function: Callable):
#         if isinstance(msg, list):
#             for sub_msg in msg:
#                 self.__add_message_listener(sub_msg, function)
#         else:
#             self.__add_message_listener(msg, function)

#     return decorator
import time


class Connection:
    def __init__(self) -> None:
        self.party = "WHOOO"
        self.message_listeners = {}
        self.receivers = Receivers()
        self.c = C()

    @property
    def testing(self):
        return self.message_listeners

    def pain(self):
        self.receivers.update_a(self)
        return

    def sad(self):
        self.c.call_method("i made it", self)


class C:
    def __init__(self) -> None:
        self.methods = {}

        @self.partay(["woo"])
        @self.timer()
        def party(self, msg, instance):
            instance.party = msg

    def partay(self, msg):
        def decorator(function):
            if isinstance(msg, list):
                for sub_msg in msg:
                    self.__add_message_listener(sub_msg, function)
            else:
                self.__add_message_listener(msg, function)

        return decorator

    def __add_message_listener(self, msg, function):
        if msg not in self.methods:
            self.methods[msg] = []

        if function not in self.methods[msg]:
            self.methods[msg].append(function)

        return

    def timer(self):
        """
        Decorator used to log the time that a sender takes to complete. Used for
        debugging purposes.

        :return: decorator
        :rtype: Callable
        """

        def decorator(function):
            def wrapper(*args):
                start_t = time.time()
                response = function(*args)
                print(f"Time taken to execute function: {time.time() - start_t}")
                return response

            return wrapper

        return decorator

    def call_method(self, msg, instance):
        self.methods["woo"][0](self, msg, instance)
        instance.testing["robot"] = msg


class Receivers:
    def __init__(self) -> None:
        pass

    def update_a(self, instance: Connection):
        instance.party = "Party HARDER"


def main():
    a = Connection()

    print(a.party)

    a.pain()

    print(a.party)

    print(a.message_listeners)

    c = C()

    a.sad()

    print(a.party)

    print(a.message_listeners)


if __name__ == "__main__":
    main()
