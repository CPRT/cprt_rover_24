import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from interfaces.srv import GetNodeStatus
from interfaces.srv import LaunchNode
from interfaces.srv import ListNodes
from interfaces.srv import ShutdownAllNodes
from interfaces.srv import StopNode
from interfaces.msg import LaunchInfo

from .node_info import NodeInfo
from .node_monitor import NodeMonitor
from . import node_launcher, node_terminator

import os
import tempfile
import json
import logging
import threading
import time
import traceback
import psutil

node_logger = logging.getLogger(__name__)
logging.basicConfig()


class NodeManager(Node):
    def __init__(self):
        super().__init__("node_manager")

        self.declare_parameter("launch_keys", None)
        self.declare_parameter("state_file_path", "~/.ros/node_manager_state.json")

        self._nodes: dict[str, NodeInfo] = {}
        self._mutex: threading.Lock = threading.Lock()

        self._persistent_state_file_ref: str = os.path.expanduser(self.get_parameter("state_file_path").value)
        os.makedirs(os.path.dirname(self._persistent_state_file_ref), exist_ok=True)


        threading.Thread(target=self._node_monitor_loop, daemon=True)

        # --- Recovery Init ----
        intended_nodes = self._load_state()
        self.get_logger().info(f"Loaded state from {self._persistent_state_file_ref}")
        self.create_timer(1.0, lambda: self._recover_nodes(intended_nodes), oneshot=True)


        self.launcher_service = self.create_service(
            LaunchNode, "/node_manager/launch_node", self._launch_node_callback
        )
        self.stopper_service = self.create_service(
            StopNode, "/node_manager/stop_node", self._stop_node_callback
        )
        self.node_status_service = self.create_service(
            GetNodeStatus,
            "/node_manager/get_node_status",
            self._get_node_status_callback,
        )
        self.node_listing_service = self.create_service(
            ListNodes, "/node_manager/list_nodes", self._list_nodes_callback
        )
        self.shutdown_service = self.create_service(
            ShutdownAllNodes, "/node_manager/shutdown_all", self._shutdown_all_callback
        )

        self.launch_files: dict[str, LaunchInfo] = {}
        for key in self.get_parameter("launch_keys").value:
            self.declare_parameter(key + ".display_name", "")
            self.declare_parameter(key + ".package", "")
            self.declare_parameter(key + ".launch_file", "")

            self.launch_files[key] = LaunchInfo()
            self.launch_files[key].display_name = self.get_parameter(
                key + ".display_name"
            ).value
            self.launch_files[key].package = self.get_parameter(key + ".package").value
            self.launch_files[key].launch_file = self.get_parameter(
                key + ".launch_file"
            ).value
            self.launch_files[key].internal_name = key

        node_logger.info("node manager is set up")
        node_logger.info(str(self.launch_files))

    def launch_node(
        self,
        name: str,
    ) -> bool:
        """
        Launch a node group.

        Parameters
        ----------
        name : str
            Internal name of the node group. Corresponds to a `LaunchInfo`.

        Returns
        -------
        bool
            True if the node was launched successfully, False otherwise.
        """

        try:
            launch_info = self.launch_files[name]
            node_info = NodeInfo(
                name,
                launch_info.package,
                None,
                launch_info.launch_file,
                # TODO: respawn control
                True,
            )
        except ValueError as error:
            node_logger.error(str(error))
            return False

        with self._mutex:
            if (
                name in self._nodes
                and self._nodes[name].process
                and self._nodes[name].process.poll() is None
            ):
                node_logger.warning(f"Node '{name}' is already running.")
                return False

            try:
                proc = node_launcher.launch_node(node_info)
                node_info.process = proc
                node_info.start_time = time.time()
                self._nodes[name] = node_info
                node_logger.info(f"Started {node_info.mode} '{name}' (PID {proc.pid}).")

                self._save_state()

            except Exception as e:
                node_logger.error(f"Error launching node '{name}': {e}")
                self._nodes.pop(name, None)
                return False

        if node_info.mode == "launch":
            threading.Thread(
                target=self._discover_children, args=(node_info,), daemon=True
            ).start()
        return True

    def _launch_node_callback(
        self, request: LaunchNode.Request, response: LaunchNode.Response
    ) -> LaunchNode.Response:
        response.success = self.launch_node(
            request.name,
        )
        if response.success:
            node_logger.info(f"Node `{request.name}` launched")
        else:
            node_logger.error(f"Node `{request.name}` failed to launch")
        return response

    def stop_node(self, name: str) -> bool:
        """
        Stop a running node or group gracefully.

        Parameters
        ----------
        name : str
            The name of the node to stop.

        Returns
        -------
        bool
            True if the node was stopped (or was not running), False otherwise.
        """
        with self._mutex:
            if name not in self._nodes:
                node_logger.warning(f"Stop requested for unknown node '{name}'.")
                return False
            node_info = self._nodes[name]
        try:
            node_terminator.terminate(node_info)
            with self._mutex:
                _ = self._nodes.pop(name, None)
            self._save_state()
            node_logger.info(f"Node '{name}' stopped.")
            return True
        except Exception as e:
            node_logger.error(f"Error stopping node '{name}': {e}")
            traceback.print_exc()
            return False

    def _stop_node_callback(
        self, request: StopNode.Request, response: StopNode.Response
    ) -> StopNode.Response:
        response.success = self.stop_node(request.name)
        return response

    def _get_node_status_nonblocking(self, name: str) -> str:
        if name not in self._nodes:
            return "stopped"
        proc = self._nodes[name].process
        return "running" if proc and proc.poll() is None else "stopped"

    def get_node_status(self, name: str) -> str:
        """
        Get the status of a managed node.

        Parameters
        ----------
        name : str
            The name of the node.

        Returns
        -------
        str
            'running', 'stopped', or 'not found'.
        """
        with self._mutex:
            return self._get_node_status_nonblocking(name)

    def _get_node_status_callback(
        self, request: GetNodeStatus.Request, response: GetNodeStatus.Response
    ) -> GetNodeStatus.Response:
        response.status = self.get_node_status(request.name)
        return response

    def list_nodes(self) -> list[LaunchInfo]:
        """
        List the names of all launchable nodes.

        Returns
        -------
        list of LaunchInfo
            List containing the information about launchable nodes.
        """
        with self._mutex:
            for launch_file in self.launch_files:
                self.launch_files[launch_file].status = (
                    self._get_node_status_nonblocking(launch_file)
                )
            return list(self.launch_files.values())

    def _list_nodes_callback(
        self, _: ListNodes.Request, response: ListNodes.Response
    ) -> ListNodes.Response:
        response.nodes = self.list_nodes()
        return response

    def shutdown_all(self) -> None:
        """
        Gracefully shut down all managed nodes.
        """
        with self._mutex:
            names = list(self._nodes.keys())
        for name in names:
            node_logger.info(f"Shutting down '{name}'.")
            _ = self.stop_node(name)

    def _shutdown_all_callback(
        self, _: ShutdownAllNodes.Request, response: ShutdownAllNodes.Response
    ) -> ShutdownAllNodes.Response:
        self.shutdown_all()
        return response

    def _node_monitor_loop(self) -> None:
        """
        Background thread that monitors nodes for unexpected shutdowns
        and schedules respawns if needed.
        """
        while True:
            with self._mutex:
                nodes_copy = self._nodes.copy()
            for name, node_info in list(nodes_copy.items()):
                proc = node_info.process
                if proc is None:
                    continue
                if proc.poll() is not None:
                    node_logger.warning(
                        f"Node '{name}' terminated unexpectedly with code {proc.poll()}."
                    )
                    if node_info.respawn:
                        node_info.retries += 1
                        delay = min(5, 2 * node_info.retries)
                        node_logger.info(
                            f"Respawning '{name}' in {delay} seconds (attempt {node_info.retries})."
                        )
                        with self._mutex:
                            _ = self._nodes.pop(name, None)
                        threading.Timer(
                            delay,
                            lambda ni=node_info: self.launch_node(
                                ni.name,
                            ),
                        ).start()
                    else:
                        node_logger.info(f"Not respawning '{name}' (respawn disabled).")
                        with self._mutex:
                            self._nodes.pop(name, None)
                        should_save_state = True
                    if should_save_state:
                        self._save_state()
                else:
                    if node_info.mode == "launch":
                        try:
                            parent = psutil.Process(proc.pid)
                            current_children = parent.children(recursive=True)
                            current_pids = {child.pid for child in current_children}
                            recorded_pids = {child.pid for child in node_info.children}
                            if recorded_pids and (recorded_pids - current_pids):
                                node_logger.warning(
                                    f"One or more child processes of '{name}' have died. Restarting group."
                                )
                                with self._mutex:
                                    self._nodes.pop(name, None)
                                threading.Timer(
                                    2.0,
                                    lambda ni=node_info: self.launch_node(
                                        ni.name,
                                    ),
                                ).start()
                        except Exception as e:
                            node_logger.error(
                                f"Error monitoring children for '{name}': {e}"
                            )
            time.sleep(1.0)

    @staticmethod
    def _discover_children(node_info: NodeInfo) -> None:
        """
        For launch groups, discover and record child processes after launch.

        Parameters
        ----------
        node_info : NodeInfo
            The node information instance.
        """
        deadline = time.time() + 5  # wait up to 5 seconds
        while time.time() < deadline:
            try:
                parent = psutil.Process(node_info.process.pid)
                children = parent.children(recursive=True)
                if children:
                    node_info.children = children
                    node_logger.info(
                        f"Discovered {len(children)} child processes for '{node_info.name}'."
                    )
                    return
            except psutil.NoSuchProcess:
                break
            time.sleep(0.5)
        node_logger.warning(
            f"No child processes discovered for '{node_info.name}' within timeout."
        )


    def _load_state(self) -> list[str]:
        try:
            with open(self._persistent_state_file_ref, 'r') as f:
                data = json.load(f)
                return data.get("intended_nodes", [])
        except FileNotFoundError:
            self.get_logger().info(f"State file not found: {self._persistent_state_file_ref}")
            return []
        except (json.JSONDecodeError, IOError) as e:
            self.get_logger().error(f"Error loading state file {self._persistent_state_file_ref}")
            # back up corrupted shit here maybe?
            return []
    
    def _save_state(self) -> None:
        with self._mutex:
            intended_nodes = list(self._nodes.keys())

        self.get_logger().debug(f"Saving state: {intended_nodes}")
        data = {"intended_nodes" : intended_nodes}

        try:
            dirname = os.path.dirname(self._persistent_state_file_ref)
            with tempfile.NamedTemporaryFile('w', dir=dirname, delete=False) as tf:
                json.dump(data, tf, indent=2)
                temp_path = tf.name
            os.replace(temp_path, self._persistent_state_file_ref)
            self.get_logger().debug(f"State saved to uhhhhh this file {self._persistent_state_file_ref}")
        except (IOError, OSError) as e:
            self.get_logger().error(f"Failed to save the state to {self._persistent_state_file_ref}")
            if 'temp_path' in locals() and os.path.exists(temp_path):
                try:
                    os.remove(temp_path)
                except OSError as remove_e:
                    self.get_logger().error(f"Failed to remove temporary file {temp_path}: {remove_e}")


    def _recover_nodes(self, intended_nodes: list[str]) -> None:
        self.get_logger().info(f"Attempting recovery launch for: {intended_nodes}")
        active_nodes_before_recovery = set(self._nodes.keys())

        nodes_to_recover = [name for name in intended_nodes if name not in active_nodes_before_recovery]

        if not nodes_to_recover:
            self.get_logger().info("No nodes require recovery launch.")
            return

        for name in nodes_to_recover:
            if name not in self.launch_files:
                self.get_logger().warning(f"Skipping recovery for '{name}': Not found in launch configurations.")
                continue

            self.get_logger().info(f"Attempting recovery launch for '{name}'...")
            success = self.launch_node(name)
            if success:
                self.get_logger().info(f"Successfully recovered '{name}'.")
            else:
                self.get_logger().error(f"Failed to recover '{name}'.")
            time.sleep(0.5)
    
    def _graceful_shutdown(self) -> None:
        self.get_logger().info("Node Manager shutting down gracefully...")
        self.shutdown_all()
        self.get_logger().info("Node Manager shutdown complete.")



def main(args=None):
    rclpy.init(args=args)
    service = NodeManager()
    executor = MultiThreadedExecutor()
    executor.add_node(service)
    executor.spin()
    service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
