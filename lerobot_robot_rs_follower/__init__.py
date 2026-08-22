"""LeRobot third-party robot plugin for RobStride RS follower.

Importing this package must be lightweight because LeRobot discovers third-party
robots by importing packages whose names start with ``lerobot_robot_``.  The
config is imported immediately so ``RobotConfig.register_subclass("rs_follower")``
runs during discovery.  The hardware implementation is imported lazily to avoid
hiding the robot type when optional runtime dependencies are missing.
"""

from .config_rs_follower import RSFollowerConfig


def __getattr__(name):
    if name == "RSFollower":
        from .rs_follower import RSFollower

        return RSFollower
    raise AttributeError(name)


__all__ = ["RSFollower", "RSFollowerConfig"]
