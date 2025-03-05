import rclpy
from rclpy.node import Node
import subprocess
import os
import signal
import sys

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

    def receive_data(self):
        if hasattr(self, 'pipe_reader') and self.pipe_reader:
            try:
                line = self.pipe_reader.readline()
                if line:
                    if line.startswith("UI_DATA:"):  # Check for the prefix
                        try:
                            data = line.replace("UI_DATA:", "")  # Remove the prefix
                            x_val = data.split("x:")[1].split(",")[0]
                            y_val = data.split("y:")[1].strip()
                            self.get_logger().info(f"Received UI Data: x={x_val}, y={y_val}")
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

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.receive_data()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()