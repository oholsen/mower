import logging
import os
import subprocess
from typing import Optional

logger = logging.getLogger(__name__)
_BINDIR = os.environ.get("BINDIR", os.environ["HOME"] + "/bin")
_STREAM = _BINDIR + "/stream-logitech.sh"
_SNAPSHOT = [_BINDIR + "/fswebcam", "-r", "1920x1080", "--no-banner"]


class Camera:

    # Logitech C920 USB 1080p mounted on servos

    def __init__(self):
        self._streamer: Optional[subprocess.Popen] = None

    def _snapshot(self, filename: str):
        logger.info("Take camera snapshot: %s", filename)
        assert self._streamer is None
        result = subprocess.run(_SNAPSHOT + [filename], capture_output=True)
        logger.debug("Snapshot result: %s", result)

    def snapshot(self, filename: str):
        if self._streamer:
            self.stop_stream()
            self._snapshot(filename)
            self.start_stream()
        else:
            self._snapshot(filename)

    def start_stream(self):
        assert self._streamer is None
        self._streamer = subprocess.Popen(_STREAM, close_fds=True)
        logger.debug("Start camera stream: %s", self._streamer.pid)

    def stop_stream(self, timeout=None):
        if self._streamer is not None:
            logger.debug("Stop camera stream: %s", self._streamer.pid)
            self._streamer.terminate()
            result = self._streamer.wait(timeout)
            logger.debug("Streamer state: %s", result)
            if result is None:
                logger.error("Failed to stop video stream")
            self._streamer = None
