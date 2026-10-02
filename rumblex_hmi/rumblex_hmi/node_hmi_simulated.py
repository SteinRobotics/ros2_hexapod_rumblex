"""Run the shared HMI node with a simulated OLED on the GUI's main thread."""

import pygame
import rclpy

from rumblex_hmi.node_hmi import NodeHmi
from rumblex_hmi.simulated import SimulatedBackend


def main(args=None):
    rclpy.init(args=args)
    backend = None
    node = None
    try:
        backend = SimulatedBackend()
        node = NodeHmi(backend)
        node.declare_parameter('simulated_voltage', 12.0)
        node.declare_parameter('simulated_current', 0.0)
        backend.power_values = lambda: (
            node.get_parameter('simulated_voltage').value,
            node.get_parameter('simulated_current').value,
        )
        clock = pygame.time.Clock()
        while rclpy.ok() and backend.process_events():
            rclpy.spin_once(node, timeout_sec=0.0)
            clock.tick(60)
    except KeyboardInterrupt:
        pass
    except pygame.error as error:
        if node is not None:
            node.get_logger().error(f'Cannot open simulated OLED GUI: {error}')
        else:
            print(f'Cannot open simulated OLED GUI: {error}')
        raise
    finally:
        try:
            if node is not None:
                try:
                    node.shutdown_callback()
                finally:
                    node.destroy_node()
        finally:
            try:
                if backend is not None:
                    backend.close()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == '__main__':
    main()
