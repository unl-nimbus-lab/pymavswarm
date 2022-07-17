from pymavswarm.utils import parse_log_file

if __name__ == "__main__":
    print(
        parse_log_file(
            "/workspaces/pymavswarm/logs/2022-07-15-19-58-44.log", "TIMESYNC"
        )
    )
