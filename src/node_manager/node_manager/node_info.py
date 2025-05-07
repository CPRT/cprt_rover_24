# node_manager/node_info.py
import subprocess
import psutil
from typing import Optional, List # Make sure List is imported

class NodeInfo:
    """
    Represents a managed node (or group) with its launch parameters and state.
    """

    def __init__(
        self,
        name: str,
        package: str,
        executable: Optional[str] = None,
        launch_file: Optional[str] = None,
        ros_args: Optional[List[str]] = None, # New field for ROS arguments
        respawn: bool = True,
    ):
        """
        Parameters
        ----------
        name : str
            Unique name for the node or group.
        package : str
            ROS2 package name.
        executable : str | None, optional
            Executable name (for "ros2 run"). Default is None.
        launch_file : str | None, optional
            Launch file name (for "ros2 launch"). Default is None.
        ros_args : list[str] | None, optional
            List of ROS arguments (e.g., ["param_name:=value", "another_arg:=val"])
            or command line arguments for 'ros2 run'. Default is None.
        respawn : bool, optional
            If True, the node will automatically restart if it dies unexpectedly.
            Default is True.

        Raises
        ------
        ValueError
        If neither or both of `executable` and `launch_file` are provided.
        """

        if executable is None and launch_file is None:
            raise ValueError(
                "Specify exactly one of 'executable' or 'launch_file' (neither were provided)."
            )
        if executable is not None and launch_file is not None:
            raise ValueError(
                "Specify exactly one of 'executable' or 'launch_file' (both were provided)."
            )

        self.name: str = name
        self.package: str = package
        self.executable: Optional[str] = executable
        self.launch_file: Optional[str] = launch_file
        self.ros_args: List[str] = ros_args if ros_args is not None else [] # Store as list
        self.respawn: bool = respawn
        self.mode: str = "run" if executable is not None else "launch"
        self.process: Optional[subprocess.Popen[str]] = None
        self.children: list[psutil.Process] = []
        self.retries: int = 0
        self.start_time: float = 0.0