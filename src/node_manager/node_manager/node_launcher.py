import subprocess

from .node_info import NodeInfo

def launch_node(node_info: NodeInfo) -> subprocess.Popen[str]:
    """
    Launch a node or group.

    Parameters
    ----------
    node_info : NodeInfo
        The node information instance.

    Returns
    -------
    subprocess.Popen[str]
        The process object representing the launched node.

    Raises
    ------
    RuntimeError
        If the process fails to launch.
    """
    if node_info.mode == "run":
        assert node_info.executable is not None
        cmd = ['ros2', 'run', node_info.package, node_info.executable]
    else:
        assert node_info.launch_file is not None
        cmd = ['ros2', 'launch', node_info.package, node_info.launch_file]
    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True  # creates a separate process group
        )
    except Exception as e:
        raise RuntimeError(f"Failed to launch '{node_info.name}': {e}")
    return proc
