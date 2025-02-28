'''
Provide the ability to query the user, via a command line interface, to know which position
given as (x,y) is desired for the 2 DOF robotic arm. This information is fed to the desiredPosition
service.
'''

import rclpy
from rclpy.node import Node
from desired_position_pkg.srv import DesiredPosition

def main():
    rclpy.init()
    node = Node('position_client')
    client = node.create_client(DesiredPosition, 'desiredPosition')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn('Waiting for desiredPosition service...')
    
    while rclpy.ok():
        try:
            x = float(input("Enter desired X position: "))
            y = float(input("Enter desired Y position: "))
        except ValueError:
            node.get_logger().error("Invalid input. Please enter numerical values.")
            continue
        
        request = DesiredPosition.Request()
        request.desired_x = x
        request.desired_y = y
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        if future.result() is not None:
            if future.result().valid_position:
                node.get_logger().info(f"Position ({x}, {y}) is valid. The robot arm will move to this position.")
                break
            else:
                node.get_logger().warn(f"Position ({x}, {y}) is invalid. Please enter a new coordinate.")
        else:
            node.get_logger().error("Failed to receive a response from the service.")
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()