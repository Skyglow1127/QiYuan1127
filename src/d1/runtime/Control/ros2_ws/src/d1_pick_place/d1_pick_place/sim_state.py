"""Synthetic state for isolated planning; refuses normal hardware ROS domain."""
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '77':
        raise SystemExit('Synthetic state requires ROS_DOMAIN_ID=77 (isolated offline planning).')
    rclpy.init()
    node = Node('d1_simulated_state')
    node.declare_parameter('positions', [0., -.5, .8, 0., .3, 0.])
    values = node.get_parameter('positions').value
    if len(values) != 6:
        raise ValueError('Need six simulated joint positions')
    pub = node.create_publisher(JointState, '/joint_states', 10)
    def tick():
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = [f'Joint{i}' for i in range(1, 7)] + ['Joint7_1', 'Joint7_2']
        msg.position = list(values) + [.02, -.02]
        pub.publish(msg)
    node.create_timer(.05, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
