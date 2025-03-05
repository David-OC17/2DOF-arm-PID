"""
UIControllerNode: ROS2 Node for Streamlit UI Communication

Launches Streamlit, uses pipes to receive UI data, and distinguishes
it from console output.

    - Launches Streamlit (`ui.py`).
    - Receives UI data via pipes.
    - Separates UI data ("UI_DATA:") from console output.
    - Logs UI data, prints console output.
    - Graceful Streamlit shutdown.

Usage:
    - Run: `ros2 run <package> ui_controller`.
    - UI data: "UI_DATA:x:<x>,y:<y>\n".
"""

import rclpy
from rclpy.node import Node
from desired_position_pkg.srv import DesiredPosition

import subprocess
import os
import signal
import sys
from typing import Tuple

class UIControllerNode(Node):
    def __init__(self):
        super().__init__('ui_controller')
        self.get_logger().info("✅ UI Controller Node Started")

        # Get the path of the Streamlit UI script
        script_path = os.path.join(os.path.dirname(__file__), "ui.py")

        # Create the pipe for data from UI to ROS2
        r, w = os.pipe()

        # Launch Streamlit in a subprocess, redirecting stdout to the pipe
        self.streamlit_process = subprocess.Popen(
            ["python3", "-m", "streamlit", "run", script_path],
            stdout=w,  # Redirect stdout to the write end of the pipe (for data to ROS2)
            stderr=sys.stderr,  # Send errors to main console.
            stdin=subprocess.DEVNULL
        )

        # Close the write end in the parent
        os.close(w)
        self.pipe_reader = os.fdopen(r, 'r')  # Turn the read end into a file object

        self.get_logger().info("✅ Streamlit UI Started")


    def send_data_to_server(self, client = "RunnableClient", values: Tuple = (0,0)):

        x , y = values
        self.get_logger().info(f"Sending data recieved from UI: x={x}, y={y}")

        #Send data to inverse kinematics server
        request = DesiredPosition.Request()
        request.desired_x = x
        request.desired_y = y
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            if future.result().valid_position:
                self.get_logger().info(f"Position ({x}, {y}) is valid. The robot arm will move to this position.")
            else:
                self.get_logger().warn(f"Position ({x}, {y}) is invalid. Please enter a new coordinate.")
        else:
            self.get_logger().error("Failed to receive a response from the service.")



    # Handle data passed from child UI process
    def receive_data(self) -> Tuple[float, float]:
        if hasattr(self, 'pipe_reader') and self.pipe_reader:
            try:
                line = self.pipe_reader.readline()
                if line:
                    if line.startswith("UI_DATA:"):  # Check for the prefix
                        try:
                            data = line.replace("UI_DATA:", "")  # Remove the prefix
                            x_val = float(data.split("x:")[1].split(",")[0])
                            y_val = float(data.split("y:")[1].strip())
                            #self.get_logger().info(f"Received UI Data: x={x_val}, y={y_val}")

                            return (x_val , y_val)



                        except (IndexError, ValueError) as e:
                            self.get_logger().warning(f"Invalid UI Data: {e}, Line: {line}")
                    else:
                        # Handle regular console output from Streamlit
                        print(line.strip()) #Print the line without the newline character.
                        
            except BrokenPipeError:
                self.get_logger().warning("Streamlit process closed, pipe broken.")
                self.pipe_reader.close()
                self.pipe_reader = None

    def destroy_node(self):
        self.get_logger().info("🔴 Shutting down Streamlit UI")
        if hasattr(self, 'streamlit_process'):
            self.streamlit_process.send_signal(signal.SIGINT)  # Gracefully stop Streamlit
            self.streamlit_process.wait()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UIControllerNode()

    client = node.create_client(DesiredPosition, 'desiredPosition')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn('Waiting for desiredPosition service...')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            values = node.receive_data()

            if(values):
                node.send_data_to_server(client, values)

            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()