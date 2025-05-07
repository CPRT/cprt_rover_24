import subprocess
import time

from .node_info import NodeInfo


def launch_node(node_info: NodeInfo) -> subprocess.Popen[str]:
    """
    Launch a node or group, including its ROS arguments.

    Parameters
    ----------
    node_info : NodeInfo
        The node information instance, containing ros_args.

    Returns
    -------
    subprocess.Popen[str]
        The process object representing the launched node.

    Raises
    ------
    RuntimeError
        If the process fails to launch or exits immediately with an error.
    """
    cmd_base = []
    if node_info.mode == "run":
        assert node_info.executable is not None
        cmd_base = ["ros2", "run", node_info.package, node_info.executable]
    else:
        assert node_info.launch_file is not None
        cmd_base = ["ros2", "launch", node_info.package, node_info.launch_file]


    cmd = cmd_base + node_info.ros_args

    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
    except FileNotFoundError as e:
        raise RuntimeError(f"Failed to launch '{node_info.name}': Command or component '{e.filename}' not found. Is ROS2 sourced and are package/executable names and arguments correct?")
    except Exception as e:
        raise RuntimeError(f"Failed to launch '{node_info.name}': {e}")

    time.sleep(0.2) 

    if proc.poll() is not None:
        stdout, stderr = proc.communicate()

        raise RuntimeError(
            f"Process '{node_info.name}' exited immediately with code {proc.returncode}."
            f" Command: '{' '.join(cmd)}'.\n" # Log the command executed
            f"STDOUT: {stdout.strip()}\n"
            f"STDERR: {stderr.strip()}"
        )
    
    return proc