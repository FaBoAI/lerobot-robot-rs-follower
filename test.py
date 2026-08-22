from lerobot_robot_rs_follower.config_rs_follower import RSFollowerConfig
from lerobot_robot_rs_follower.rs_follower import RSFollower

cfg = RSFollowerConfig(id="black", channel="can0")
robot = RSFollower(cfg)
print(robot.action_features)
# robot.calibrate()
