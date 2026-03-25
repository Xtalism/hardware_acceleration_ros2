#!/usr/bin/env python3
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuVisualizer(Node):
    def __init__(self):
        super().__init__("imu_visualizer")
        self.subscription = self.create_subscription(
            Imu, "imu/data_raw", self.listener_callback, 10
        )
        # Store latest acceleration [x, y, z]
        self.accel = [0.0, 0.0, 9.8]  # Default approximately to gravity

    def listener_callback(self, msg):
        self.accel[0] = msg.linear_acceleration.x
        self.accel[1] = msg.linear_acceleration.y
        self.accel[2] = msg.linear_acceleration.z


def main(args=None):
    rclpy.init(args=args)
    node = ImuVisualizer()

    # Setup Matplotlib for interactive rendering (non-blocking)
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    try:
        # Loop to process ROS callbacks and update the plot
        while rclpy.ok():
            # Process ROS 2 callbacks
            rclpy.spin_once(node, timeout_sec=0.01)

            # Clear the axis for the new frame
            ax.cla()

            # Fixed axis limits to prevent the graph from resizing constantly (in m/s^2)
            limit = 20.0
            ax.set_xlim([-limit, limit])
            ax.set_ylim([-limit, limit])
            ax.set_zlim([-limit, limit])

            ax.set_xlabel("X (m/s^2)")
            ax.set_ylabel("Y (m/s^2)")
            ax.set_zlabel("Z (m/s^2)")
            ax.set_title("Live Accelerometer Data")

            # Draw a 3D arrow from origin (0,0,0) to the acceleration values
            ax.quiver(
                0,
                0,
                0,
                node.accel[0],
                node.accel[1],
                node.accel[2],
                color="red",
                arrow_length_ratio=0.1,
                linewidth=2,
            )

            # Draw a faint dot at origin for visual reference
            ax.scatter(0, 0, 0, color="black", s=20)

            # Pause to render the plot GUI events
            plt.pause(0.05)

    except KeyboardInterrupt:
        # Exit cleanly on Ctrl+C
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close()


if __name__ == "__main__":
    main()
