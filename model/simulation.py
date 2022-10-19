import matplotlib.pyplot as plt
from itertools import count
import signal
# import readchar

from visualization.visualizer_matplotlib import Visualizer_Matplotlib as Visualizer_MPL



        


class Simulation():
    def __init__(self, model, control, path, visualizer_type="matplolib"):
        self.run_sim = True

        if visualizer_type == "matplolib":
            self._vis = Visualizer_MPL(path, model.robot_state(), model.robot_parameters())
       
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signum, frame):
        self.run_sim = False
        self._vis.close()

    def run(self):
        print("Simulation started...")


        while self.run_sim:
            # print(i)
            self._vis.plot()
            # plt.pause(1e-9)

        plt.show()