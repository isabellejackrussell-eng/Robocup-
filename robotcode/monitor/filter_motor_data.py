import io
import os
from datetime import datetime

from platformio.device.monitor.filters.base import DeviceMonitorFilterBase


class MotorDataTextFile(DeviceMonitorFilterBase):
    NAME = "motor_data"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._output = None
        self._buffer = ""

    def __call__(self):
        output_path = os.path.join(self.project_dir, "motor_test_data.txt")
        if os.path.isfile(output_path) and os.path.getsize(output_path) > 0:
            timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            archive_path = os.path.join(
                self.project_dir, "motor_test_data-%s.txt" % timestamp
            )
            os.replace(output_path, archive_path)
            print("--- Archived previous motor data to %s" % archive_path)
        self._output = io.open(output_path, "w", encoding="utf-8")
        print("--- Recording motor test data to %s" % output_path)
        return self

    def __del__(self):
        if self._output:
            self._output.close()

    def rx(self, text):
        self._buffer += text
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            line = line.rstrip("\r")
            if line.startswith("record,") or line.startswith("DATA,"):
                self._output.write(line + "\n")
                self._output.flush()
        return text
