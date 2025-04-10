import os
import signal
import subprocess

from .node_info import NodeInfo


def terminate(node_info: NodeInfo, grace_timeout: float = 5.0) -> None:
    """
    Terminate the process associated with the node.

    Parameters
    ----------
    node_info : NodeInfo
        The node information instance.
    grace_timeout : float, optional
        Time in seconds to wait for graceful termination before sending SIGKILL.
        Default is 5.0 seconds.

    Raises
    ------
    RuntimeError
        If an error occurs during termination.
    """
    if node_info.process is None:
        return
    proc = node_info.process
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGINT)
        try:
            _ = proc.wait(timeout=grace_timeout)
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGKILL)
            _ = proc.wait()
    except Exception as e:
        raise RuntimeError(f"Error terminating '{node_info.name}': {e}")
