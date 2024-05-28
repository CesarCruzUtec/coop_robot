#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import os

from std_msgs.msg import String
from matplotlib.gridspec import GridSpec
from matplotlib.animation import FuncAnimation


class PlotData:
    def __init__(self):
        print("Initializing plotdata")
        self.initPlot = False
        self.plotv = [[] for _ in range(10)]
        # plotv[0] = time
        # plotv[1:4] = reference x, y, theta
        # plotv[4:7] = current x, y, theta
        # plotv[7:10] = error x, y, theta
        self.fig = plt.figure(figsize=(12, 6))

    def plot_init(self):
        gs1 = GridSpec(3, 2, wspace=0.15, hspace=0.1)
        self.ax1 = self.fig.add_subplot(gs1[:, 0])
        self.ax2 = self.fig.add_subplot(gs1[0, 1])
        self.ax3 = self.fig.add_subplot(gs1[1, 1], sharex=self.ax2)
        self.ax4 = self.fig.add_subplot(gs1[2, 1], sharex=self.ax2)
        self.ax1.set_xlim(-1, 3)
        self.ax1.set_ylim(-1, 3)
        self.ax1.grid()
        self.ax2.grid()
        self.ax3.grid()
        self.ax4.grid()
        self.ax1.set_xlabel("x(m)")
        self.ax1.set_ylabel("y(m)")
        self.ax2.set_ylabel("x(m)")
        self.ax3.set_ylabel("y(m)")
        self.ax4.set_ylabel("theta(rad)")
        self.ax4.set_xlabel("time(s)")
        self.ax2.tick_params(labelbottom=False)
        self.ax3.tick_params(labelbottom=False)

        self.ln1 = self.ax1.plot([], [], "b")[0]

        self.ln2_c, self.ln2_r, self.ln2_e = self.ax2.plot(
            [], [], "b", [], [], "r", [], [], "g"
        )

        self.ln3_c, self.ln3_r, self.ln3_e = self.ax3.plot(
            [], [], "b", [], [], "r", [], [], "g"
        )

        self.ln4_c, self.ln4_r, self.ln4_e = self.ax4.plot(
            [], [], "b", [], [], "r", [], [], "g"
        )

    def callback(self, data):
        data = data.data
        if not data:
            return

        inData = data.split(",")
        if any([x == "" for x in inData]):
            return

        status = inData[-1]
        if status == "end":
            self.plotv = [[] for _ in range(10)]
            return

        values = [float(x) for x in inData[:-1]]
        for i in range(10):
            self.plotv[i].append(values[i])

    def update_plot(self, _):
        # os.system("clear")
        if not self.plotv[0]:
            # print("No data")
            return

        if not self.initPlot:
            print("Initializing plot")
            self.initPlot = True
            self.initTime = self.plotv[0][0]
            self.ax1.plot(self.plotv[1][0], self.plotv[2][0], "ro", markersize=2)

        self.plotv[0][-1] -= self.initTime  # Normalize time
        print("Time: {0:>6.4f} s".format(self.plotv[0][-1]))
        print("Reference: {0:>6.4f} m, {1:>6.4f} m | {2:>6.4f} rad".format(*self.plotv[1:4][-1]))

        self.ln1.set_data(self.plotv[4], self.plotv[5])

        self.ln2_c.set_data(self.plotv[0], self.plotv[4])
        self.ln2_r.set_data(self.plotv[0], self.plotv[1])
        self.ln2_e.set_data(self.plotv[0], self.plotv[7])

        self.ln3_c.set_data(self.plotv[0], self.plotv[5])
        self.ln3_r.set_data(self.plotv[0], self.plotv[2])
        self.ln3_e.set_data(self.plotv[0], self.plotv[8])

        self.ln4_c.set_data(self.plotv[0], self.plotv[6])
        self.ln4_r.set_data(self.plotv[0], self.plotv[3])
        self.ln4_e.set_data(self.plotv[0], self.plotv[9])

        # self.ax1.plot(self.plotv[4], self.plotv[5], "b")

        # self.ax2.plot(self.plotv[0], self.plotv[4], "b")
        # self.ax2.plot(self.plotv[0], self.plotv[1], "r")
        # self.ax2.plot(self.plotv[0], self.plotv[7], "g")

        # self.ax3.plot(self.plotv[0], self.plotv[5], "b")
        # self.ax3.plot(self.plotv[0], self.plotv[2], "r")
        # self.ax3.plot(self.plotv[0], self.plotv[8], "g")

        # self.ax4.plot(self.plotv[0], self.plotv[6], "b")
        # self.ax4.plot(self.plotv[0], self.plotv[3], "r")
        # self.ax4.plot(self.plotv[0], self.plotv[9], "g")


if __name__ == "__main__":
    rospy.init_node("plotdata", disable_signals=True)
    pd = PlotData()

    rospy.Subscriber("/data", String, pd.callback)

    ani = FuncAnimation(pd.fig, pd.update_plot, init_func=pd.plot_init, interval=100)
    plt.show(block=True)
