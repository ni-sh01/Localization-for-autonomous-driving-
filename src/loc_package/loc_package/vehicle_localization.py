import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TwistStamped
from mocap_msgs.msg import RigidBodies
import random


class LocNode(Node):
    def __init__(self):
        super().__init__('vehicle_localization')

        # Subscriber to /pose_modelcars
        self.subscription = self.create_subscription(
            RigidBodies,
            '/pose_modelcars',
            self.pose_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher to /ego_pose (position and orientation)
        self.publisher_ego_pose = self.create_publisher(Pose, '/ego_pose', 10)

        # Publisher to /ego_twist (dummy velocity)
        self.publisher_ego_twist = self.create_publisher(TwistStamped, '/ego_twist', 10)

    def pose_callback(self, msg):
        """
        Callback function for the pose subscription. It extracts the position and orientation
        of rigid body '8' and publishes it. It also publishes a dummy velocity.
        """
        found_ego = False

        for rigid_body in msg.rigidbodies:
            if str(rigid_body.rigid_body_name) == '8':
                found_ego = True

                # Extract position and orientation
                ego_pose = Pose()
                ego_pose.position = rigid_body.pose.position
                ego_pose.orientation = rigid_body.pose.orientation

                # Publish to /ego_pose
                self.publisher_ego_pose.publish(ego_pose)
                self.get_logger().info(f"Published position and orientation of rigid body '8'.")

                # Publish dummy velocity to /ego_twist
                self.publish_dummy_velocity()
                break  # Exit once '8' is found

        if not found_ego:
            self.get_logger().warn("Rigid body '8' not found in the message.")

    def publish_dummy_velocity(self):
        """
        Publish dummy linear and angular velocity values to the /ego_twist topic.
        - Linear: Random x, y; z = 0.0
        - Angular: Random x, y; z = 0.0
        """
        ego_twist = TwistStamped()
        ego_twist.header.stamp = self.get_clock().now().to_msg()
        ego_twist.header.frame_id = "map"

        # Generate random dummy velocity values for x and y, keep z as zero
        ego_twist.twist.linear.x = random.uniform(-1.0, 1.0)
        ego_twist.twist.linear.y = random.uniform(-1.0, 1.0)
        ego_twist.twist.linear.z = 0.0  # No vertical movement

        ego_twist.twist.angular.x = random.uniform(-0.5, 0.5)
        ego_twist.twist.angular.y = random.uniform(-0.5, 0.5)
        ego_twist.twist.angular.z = 0.0  # No rotation around z-axis

        self.publisher_ego_twist.publish(ego_twist)
        self.get_logger().info(f"Published dummy velocity: linear(x={ego_twist.twist.linear.x}, y={ego_twist.twist.linear.y}), "
                               f"angular(x={ego_twist.twist.angular.x}, y={ego_twist.twist.angular.y})")


def main(args=None):
    rclpy.init(args=args)
    node = LocNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
