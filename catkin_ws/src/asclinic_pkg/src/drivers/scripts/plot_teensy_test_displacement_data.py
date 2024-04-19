import json
from matplotlib import pyplot as plt
from pathlib import Path


class DataManager:
    def __init__(self):
        self._log_dir = Path("./log")
        if not self._log_dir.is_dir():
            raise NotADirectoryError
        log_file = self._log_dir / "logfile_060721.json"
        if not log_file.exists():
            raise FileNotFoundError
        self._data = self._read_data(log_file)

    def _read_data(self, log_file) -> dict:
        json_string = open(log_file).read()
        return json.loads(json_string)

    def approximate_total_number_of_edges_of_motor(self, index):
        if index == 1:
            return sum(self.displacement1s)
        elif index == 2:
            return sum(self.displacement2s)
        else:
            raise IndexError

    def dumps_data(self):
        print(json.dumps(self._data))

    def plot(self, filename=None):
        fig, (ax1, ax2) = plt.subplots(2, 1)
        for axis, displacements, approximate_total_number_of_edges_of_motor in zip(
            [ax1, ax2],
            [self.displacement1s, self.displacement2s],
            [
                self.approximate_total_number_of_edges_of_motor(1),
                self.approximate_total_number_of_edges_of_motor(2),
            ],
        ):
            delta_ts = self.delta_ts_between_measurements
            delta_ts_between_edges = [
                delta_t / displacement
                for delta_t, displacement in zip(delta_ts, displacements)
            ]
            delta_ts_between_edges = [
                delta_t
                for delta_t, displacement in zip(delta_ts_between_edges, displacements)
                for _ in range(displacement)
            ]
            assert (
                len(delta_ts_between_edges)
                == approximate_total_number_of_edges_of_motor
            )
            n, bins, patches = axis.hist(delta_ts_between_edges, 100, facecolor="g")
            title_segments = [
                "Approximate time difference between each edges",
                f"duration = {self._data['duration'] / 1000000}",
                f"speed = {self._data['speed']}",
                f"mode = {self._data['mode']} (0 for rotation, 1 for forawrd)",
                f"polling delta t = {self._data['pollingDeltaT']}ms",
            ]
            axis.set(title="; ".join(title_segments))
            # axis.set(title="Approximate time difference between each edges")
            axis.set_xlabel("Delta t (second)")
            axis.set_ylabel("Number of edges")
        if filename is not None and isinstance(filename, str):
            plt.savefig(filename)
        plt.show()

    @property
    def displacement1s(self):
        # `[5:]` is just a temporary hack to ``clean" the data
        return [
            measurement["displacement1"] for measurement in self._data["results"][5:]
        ]

    @property
    def displacement2s(self):
        # `[5:]` is just a temporary hack to ``clean" the data
        return [
            measurement["displacement2"] for measurement in self._data["results"][5:]
        ]

    @property
    def delta_ts_between_measurements(self):
        # `[5:]` is just a temporary hack to ``clean" the data
        return [measurement["deltaT"] for measurement in self._data["results"][5:]]


if __name__ == "__main__":
    manager = DataManager()
    # print(manager.displacement2s)
    # print(manager.delta_ts_between_measurements)
    print(manager.approximate_total_number_of_edges_of_motor(1))
    manager.plot()
