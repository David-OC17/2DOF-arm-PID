'''
Provide the ability to determine if some (x,y) desired end-effector position is reachable, and
calcualte the inverse kinematics to that point, outputing some (theta1, theta2) for the angles
of the actuators in the 2 DOF robotic arm. This works as a service.
'''

import rclpy
from rclpy.node import Node
from desired_position_pkg.srv import DesiredPosition
from std_msgs.msg import Float64MultiArray

class InverseKinematicsService(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_service')
        self.srv = self.create_service(DesiredPosition, 'desiredPosition', self.handle_position_request)
        self.publisher = self.create_publisher(Float64MultiArray, 'desired_joint', 10)
        self.get_logger().info("Inverse Kinematics Service is ready.")
    
    def handle_position_request(self, request, response):
        x, y = request.desired_x, request.desired_y
        self.get_logger().info(f"Received request for position ({x}, {y})")
        
        response.valid_position = self.validate_position(x, y)
        
        if response.valid_position:
            theta1, theta2 = self.calculate_joint_angles(x, y)
            self.publish_joint_angles(theta1, theta2)
            self.get_logger().info(f"Position ({x}, {y}) is valid. Sending joint angles ({theta1}, {theta2}).")
        else:
            self.get_logger().warn(f"Position ({x}, {y}) is invalid. Request rejected.")
        
        return response
    
    def validate_position(self, x, y):
        return (x**2 + y**2) <= 25  # Example: valid if within a 5-unit radius
    
    def calculate_joint_angles(self, x, y):
        # Placeholder for inverse kinematics calculations
        theta1 = x * 0.1  # Example placeholder calculation
        theta2 = y * 0.1  # Example placeholder calculation
        return theta1, theta2
    
    def publish_joint_angles(self, theta1, theta2):
        msg = Float64MultiArray()
        msg.data = [theta1, theta2]
        self.publisher.publish(msg)
        self.get_logger().info(f"Published joint angles: ({theta1}, {theta2})")

def main():
    rclpy.init()
    node = InverseKinematicsService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
