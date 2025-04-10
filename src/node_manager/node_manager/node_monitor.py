import threading
import time
from typing import Callable

import psutil

from .node_info import NodeInfo


class NodeMonitor:
    """
    Periodically monitors nodes for unexpected shutdowns or missing child processes.
    """

    def __init__(self, check_interval: float = 1.0):
        """
        Initialize the NodeMonitor.

        Parameters
        ----------
        check_interval : float, optional
            Interval in seconds between checks. Default is 1.0.
        """
        self.check_interval: float = check_interval

    def monitor_nodes(
        self,
        nodes: dict[str, NodeInfo],
        respawn_callback: Callable[[NodeInfo], None],
        logger: Callable[[str], None],
    ) -> None:
        """
        Monitor nodes for unexpected termination or missing child processes.

        Parameters
        ----------
        nodes : dict of {str: NodeInfo}
            Dictionary of managed nodes.
        respawn_callback : callable
            Callback to call for respawning a node.
        logger : callable
            Function to log messages.
        """
        while True:
            time.sleep(self.check_interval)
            for name in list(nodes.keys()):
                node_info = nodes.get(name)
                if node_info is None or node_info.process is None:
                    continue
                proc = node_info.process
                # Check main process status.
                if proc.poll() is not None:
                    logger(
                        f"Node '{name}' terminated unexpectedly with code {proc.poll()}."
                    )
                    if node_info.respawn:
                        node_info.retries += 1
                        delay = min(5, 2 * node_info.retries)
                        logger(
                            f"Respawning '{name}' in {delay} seconds (attempt {node_info.retries})."
                        )
                        _ = nodes.pop(name)
                        threading.Timer(
                            delay, lambda ni=node_info: respawn_callback(ni)
                        ).start()
                    else:
                        logger(f"Not respawning '{name}' (respawn disabled).")
                        _ = nodes.pop(name)
                else:
                    # For launch groups, check if expected child processes are still alive.
                    if node_info.mode == "launch":
                        try:
                            parent = psutil.Process(proc.pid)
                            current_children = parent.children(recursive=True)
                            current_pids = {child.pid for child in current_children}
                            recorded_pids = {child.pid for child in node_info.children}
                            if recorded_pids and (recorded_pids - current_pids):
                                logger(
                                    f"One or more child processes of '{name}' died unexpectedly. Restarting group."
                                )
                                _ = nodes.pop(name)
                                threading.Timer(
                                    2.0, lambda ni=node_info: respawn_callback(ni)
                                ).start()
                        except Exception as e:
                            logger(f"Error monitoring children for '{name}': {e}")
