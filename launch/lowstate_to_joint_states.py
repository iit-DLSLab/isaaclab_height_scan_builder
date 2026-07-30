#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from unitree_go.msg import LowState


# Ordine dei motori nel messaggio Unitree LowState:
# FR, FL, RR, RL
JOINT_NAMES = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",

    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",

    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",

    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
]


class LowStateToJointStates(Node):

    def __init__(self):
        super().__init__("lowstate_to_joint_states")

        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.subscription = self.create_subscription(
            LowState,
            "/lowstate",
            self.lowstate_callback,
            10,
        )

        self.get_logger().info(
            "Converting /lowstate to /joint_states"
        )

    def lowstate_callback(self, msg: LowState) -> None:
        if len(msg.motor_state) < 12:
            self.get_logger().warning(
                f"LowState contains only {len(msg.motor_state)} motors"
            )
            return

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = JOINT_NAMES

        joint_state.position = [
            float(msg.motor_state[i].q)
            for i in range(12)
        ]

        joint_state.velocity = [
            float(msg.motor_state[i].dq)
            for i in range(12)
        ]

        joint_state.effort = [
            float(msg.motor_state[i].tau_est)
            for i in range(12)
        ]

        self.publisher.publish(joint_state)


def main() -> None:
    rclpy.init()

    node = LowStateToJointStates()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()