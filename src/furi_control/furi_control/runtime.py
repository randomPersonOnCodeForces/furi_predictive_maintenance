"""Shared single-threaded ROS node lifecycle with cooperative signal shutdown."""

import signal

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions


def run_node(node_type, args=None, stop=None):
    """Finish in-flight callbacks before destroying publishers and the ROS context."""
    requested = False

    def request_shutdown(signum, frame):
        nonlocal requested
        requested = True

    # Raising KeyboardInterrupt inside Humble's pybind message conversion can
    # become a RuntimeError. Only set a flag in the handlers; unwind between spins.
    previous = {signum: signal.signal(signum, request_shutdown)
                for signum in (signal.SIGINT, signal.SIGTERM)}
    node = None
    executor = None
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = node_type()
        executor = SingleThreadedExecutor(context=node.context)
        executor.add_node(node)
        while node.context.ok() and not requested:
            executor.spin_once(timeout_sec=0.1)
    except ExternalShutdownException:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            if node.context.ok() and stop is not None:
                stop(node)
            node.destroy_node()
        rclpy.try_shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)
