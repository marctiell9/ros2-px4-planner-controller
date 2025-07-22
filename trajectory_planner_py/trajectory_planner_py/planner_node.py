#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3Stamped, PoseStamped, TwistStamped, AccelStamped

from trajectory_planner_py.min_snap_trajectory_generators import (
    optimizeTrajectory,
    YawMinAccTrajectory,
)
from trajectory_planner_py.waypoints import figure_8


class TrajectoryNode(Node):

    def __init__(self) -> None:
        super().__init__("trajectory_generator")

        self._pose_pub = self.create_publisher(PoseStamped, "/target_pose", 10)
        self._twist_pub = self.create_publisher(TwistStamped, "/target_twist", 10)
        self._accel_pub = self.create_publisher(AccelStamped, "/target_accel", 10)

        self.create_subscription(Vector3Stamped, "/att_rpy", self._rpy_cb, 10)
        self.create_subscription(Vector3Stamped, "/position", self._pos_cb, 10)

        self._curr_pos = np.zeros(3)   # current vehicle position  (x,y,z)
        self._curr_rpy = np.zeros(3)   # current vehicle attitude (roll, pitch, yaw)

        # trajectory generators
        t_wp, xyz_wp, rpy_wp = figure_8()
        self._xyz_traj = optimizeTrajectory(
            xyz_wp,
            t_wp,
            optim_target="poly-coeff",
            poly_order=7,
            floating_cubes=None,
            t_cubes=None,
        )
        self._yaw_traj = YawMinAccTrajectory(
            yaw_waypoints=rpy_wp[:, 2],
            t_waypoints=t_wp,
        )

        self._start_ns = self.get_clock().now().nanoseconds
        update_hz = 100.0
        self.create_timer(1.0 / update_hz, self._on_timer)

    def _on_timer(self) -> None:
        t_now = (self.get_clock().now().nanoseconds - self._start_ns) * 1e-9

        pos, vel, acc, _, _ = self._xyz_traj.eval(t_now)
        yaw, _, _ = self._yaw_traj.eval(t_now, des_pos=pos, curr_pos=self._curr_pos)
        target_rpy = np.array([0.0, 0.0, yaw])

        self._publish_pose(pos, target_rpy)
        self._publish_twist(vel)
        self._publish_accel(acc)

        self.get_logger().info(f"publishing commands: position={pos.round(3)}, attitude={target_rpy.round(3)}")

    def _rpy_cb(self, msg: Vector3Stamped) -> None:
        """Store the vehicle’s current roll-pitch-yaw."""
        self._curr_rpy[:] = (msg.vector.x, msg.vector.y, msg.vector.z)

    def _pos_cb(self, msg: Vector3Stamped) -> None:
        """Store the vehicle’s current XYZ position."""
        self._curr_pos[:] = (msg.vector.x, msg.vector.y, msg.vector.z)

    def _publish_twist(self, lin_vel: np.ndarray) -> TwistStamped:
        twist = TwistStamped()
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = lin_vel
        self._twist_pub.publish(twist)
        return twist

    def _publish_accel(self, lin_acc: np.ndarray) -> AccelStamped:
        accel = AccelStamped()
        accel.accel.linear.x, accel.accel.linear.y, accel.accel.linear.z = lin_acc
        self._accel_pub.publish(accel)
        return accel

    def _publish_pose(self, xyz: np.ndarray, rpy: np.ndarray) -> PoseStamped:
        quat_xyzw = Rotation.from_euler("XYZ", rpy).as_quat()  # (x,y,z,w)
        PoseStamped().pose.position.x, PoseStamped().pose.position.y, PoseStamped().pose.position.z = xyz
        PoseStamped().pose.orientation.w = quat_xyzw[3]
        PoseStamped().pose.orientation.x = quat_xyzw[0]
        PoseStamped().pose.orientation.y = quat_xyzw[1]
        PoseStamped().pose.orientation.z = quat_xyzw[2]
        self._pose_pub.publish(PoseStamped())
        return PoseStamped()

def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
