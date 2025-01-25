import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from mocap_msgs.msg import RigidBodies

class LocNode(Node):
    def __init__(self):
        super().__init__('loc_node')

        # Subscriber to /pose_modelcars
        self.subscription = self.create_subscription(
            RigidBodies,
            '/pose_modelcars',
            self.pose_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher to /team_elite_ego_pose
        self.publisher = self.create_publisher(Pose, '/team_elite_ego_pose', 10)

    def pose_callback(self, msg):
        for rigid_body in msg.rigidbodies:
            if rigid_body.rigid_body_name == '8':
                # Extract position and orientation
                ego_pose = Pose()
                ego_pose.position = rigid_body.pose.position
                ego_pose.orientation = rigid_body.pose.orientation

                # Publish to /team_elite_ego_pose
                self.publisher.publish(ego_pose)
                self.get_logger().info(f"Published position and orientation of rigid body '8': {ego_pose}")
                return  # Exit once '8' is found

        self.get_logger().warn("Rigid body '8' not found in the message.")

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

