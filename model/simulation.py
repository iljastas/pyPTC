import matplotlib.pyplot as plt
from itertools import count
import signal
# import readchar

from visualization.visualizer_matplotlib import Visualizer_Matplotlib as Visualizer_MPL



        


class Simulation():
    def __init__(self, model, control, path, visualizer_type="matplolib"):
        self.run_sim = True
        self.path = path
        self.control = control
        self.feedforward = None

        if visualizer_type == "matplolib":
            self.vis = Visualizer_MPL(path, model.robot_state(), model.robot_parameters())
       
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signum, frame):
        self.run_sim = False
        self.vis.close()

    def run(self):
        print("Simulation started...")


        while self.run_sim:
            self.path.execute(lookahead_length=self.control.lah_distance())
            
            self.control.execute()
            # self.feedforward.execute()
            
            self.model.step()
            
            self.vis.plot()
            
            # plt.pause(1e-9)

        plt.show()