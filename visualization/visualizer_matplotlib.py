import matplotlib.pyplot as plt
import matplotlib
import numpy as np

from .plot_vehicle import plot_acker, plot_skid, plot_arti

class Visualizer_Matplotlib:
    def __init__(self, path_numpy_3xN, robot_state=None, robot_parameters=None):
        self.fig, self.ax = plt.subplots(1,1)
        plt.show(block=False) # for custom pause, so that matplolib-window do not pop to front on each update

        self._path = path_numpy_3xN
        self._desPath_xy = np.array([self._path.x(), self._path.y()])
        self._robot_params = robot_parameters
        self._robot_state = robot_state

    def set_robotState(self, robot_state):
        self._robot_state = robot_state

    def close(self):
        plt.close(self.fig)

    def pause(self, interval): # custom pause, so that matplolib-window do not pop to front on each update
        backend = plt.rcParams['backend']
        if backend in matplotlib.rcsetup.interactive_bk:
            figManager = matplotlib._pylab_helpers.Gcf.get_active()
            if figManager is not None:
                canvas = figManager.canvas
                if canvas.figure.stale:
                    canvas.draw()
                canvas.start_event_loop(interval)
                return

    def plot_path(self, path_2xN):
        self.ax.plot(path_2xN[0,:], path_2xN[1,:], color="tab:red", linewidth=2)
        

    def plot(self):
        # Plot desired and driven path
        self.plot_path(self._desPath_xy)
        

        # Plot robot/vehicle
        if self._robot_params.type() == "Ackermann":
            state = self._robot_state.as_dict()
            plot_acker(self.ax, state["x"], state["y"], state["yaw"], state["steer_front"])
        elif self._robot_params.type() == "Skid":
            pass
        elif self._robot_params.type() == "Articulated":
            pass
        else:
            print("Plotting is not possible because type in parameters is wrong!")

        self.pause(1e-9)