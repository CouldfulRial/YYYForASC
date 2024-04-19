import itertools
import json
from matplotlib import pyplot as plt
from pathlib import Path


class DataManager:
    def __init__(self):
        self._log_dir = Path("./log")
        if not self._log_dir.is_dir():
            raise NotADirectoryError
        log_file = self._log_dir / "logfile.json"
        if not log_file.exists():
            raise FileNotFoundError
        self._data = self._read_data(log_file)

    def _filter_by_consecutive_edges(self, delta_ts, event_types):
        """
        Filters the data by looking at consecutive rising/falling edges (sure errors).
        """
        iterator = itertools.groupby(zip(delta_ts, event_types), lambda x: x[1])
        normal_delta_ts = []
        abnormal_delta_ts = []
        for key, group in iterator:
            consecutive_events = list(group)
            if len(consecutive_events) > 1:
                abnormal_delta_ts.extend([event[0] for event in consecutive_events[1:]])
            normal_delta_ts.append(consecutive_events[0][0])
        return normal_delta_ts, abnormal_delta_ts

    def _get_delta_ts_of_line(self, index):
        return self._data["results"][index]["deltaT"][1:]

    def _get_event_types_of_line(self, index):
        return self._data["results"][index]["eventTypes"][1:]

    def _read_data(self, log_file) -> dict:
        """
        Reads the data from the log file into a dict, and returns it.
        """
        json_string = open(log_file).read()
        return json.loads(json_string)

    def plot(self, filename):
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
        axes = [ax1, ax2, ax3, ax4]
        for i in range(4):
            delta_ts = self._get_delta_ts_of_line(i)
            event_types = self._get_event_types_of_line(i)
            normal_delta_ts, abnormal_delta_ts = self._filter_by_consecutive_edges(
                delta_ts, event_types
            )
            n, bins, patches = axes[i].hist(abnormal_delta_ts, 100, facecolor="r")
            n, bins, patches = axes[i].hist(normal_delta_ts, bins, fc=(0, 0, 1, 0.5))
            axes[i].set(title="Line " + str(i))
            axes[i].set_xlabel("Delta t (second)")
            axes[i].set_ylabel("Number of events")
        plt.savefig(filename)
        plt.show()

    def minimum_number_of_missed_events_of_line(self, index):
        return self._data["results"][index]["minNumberOfMissedEvents"]

    def number_of_measured_events_of_line(self, index):
        return self._data["results"][index]["numberOfEvents"]

    @property
    def test_speed(self):
        return self._data["speed"]


if __name__ == "__main__":
    manager = DataManager()
    manager.plot("log/delta_t.png")
    # print(manager.test_speed)
    # print(manager.number_of_measured_events_of_line(0))
    # print(manager.minimum_number_of_missed_events_of_line(0))
