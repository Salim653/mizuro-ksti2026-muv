from core import Command, Observation


class Controller:
    def __init__(self, max_speed: float, steer_limit: float) -> None:
        self.max_speed, self.steer_limit = max_speed, steer_limit

    def command(self, obs: Observation) -> Command:
        if obs.lane_error_m is None or obs.heading_error_rad is None or obs.confidence <= 0:
            return Command(reason="lane estimate unavailable")
        steer = max(-self.steer_limit, min(self.steer_limit,
                     0.8 * obs.heading_error_rad + 1.2 * obs.lane_error_m))
        return Command(speed_mps=self.max_speed, steering_rad=steer, reason="lane tracking")
