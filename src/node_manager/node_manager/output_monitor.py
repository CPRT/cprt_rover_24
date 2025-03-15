import subprocess
import threading
from typing import Callable


class OutputMonitor:
    """
    Captures stdout and stderr from a subprocess and logs the output.
    """

    def start_capture(
        self,
        name: str,
        proc: subprocess.Popen[str],
        log_info: Callable[[str], None],
        log_error: Callable[[str], None],
    ) -> None:
        """
        Start capturing output from the process.

        Parameters
        ----------
        name : str
            The name of the node.
        proc : subprocess.Popen[str]
            The process whose output is to be captured.
        log_info : callable
            Function to log informational messages.
        log_error : callable
            Function to log error messages.
        """
        threading.Thread(
            target=self._capture, args=(name, proc, log_info, log_error), daemon=True
        ).start()

    @staticmethod
    def _capture(
        name: str,
        proc: subprocess.Popen[str],
        log_info: Callable[[str], None],
        log_error: Callable[[str], None],
    ) -> None:
        """
        Capture and log the output of the process.

        Parameters
        ----------
        name : str
            The name of the node.
        proc : subprocess.Popen[str]
            The process whose output is to be captured.
        log_info : callable
            Function to log informational messages.
        log_error : callable
            Function to log error messages.
        """
        try:
            if proc.stdout:
                for line in iter(proc.stdout.readline, ""):
                    if line:
                        log_info(f"[{name} STDOUT] {line.strip()}")
            if proc.stderr:
                for line in iter(proc.stderr.readline, ""):
                    if line:
                        log_error(f"[{name} STDERR] {line.strip()}")
        except Exception as e:
            log_error(f"Error capturing output for '{name}': {e}")
