import re
import matplotlib.pyplot as plt
import time
import matplotlib.animation as animation
def read_elements(file_name):
    vs = []
    vl = []
    vlb = []
    vrb = []
    with open(file_name, 'r') as file:
        lines = file.readlines()
        for line in lines:
            elements = re.findall('s\[(.*?)\] l\[(.*?)\].*?left_edge\[(.*?)\] right_edge\[(.*?)\]', line)
            for ele in elements:
                vs.append(float(ele[0]))
                vl.append(float(ele[1]))
                vlb.append(float(ele[2]))
                vrb.append(float(ele[3]))
    return vs, vl, vlb, vrb


def plot_dyn():
    fig, ax = plt.subplots()
    ax.set_xlim(0, 100.0)
    ax.set_ylim(-5, 5.0)

    lnp, = ax.plot([], [], animated=True)
    lnlb, = ax.plot([], [], animated=True)
    lnrb, = ax.plot([], [], animated=True)

    def update(frame):
        time.sleep(0.1)
        vs, vl, vlb, vrb = read_elements("traj.txt")
        lnp.set_data(vs, vl)
        lnlb.set_data(vs, vlb)
        lnrb.set_data(vs, vrb)
        return lnp, lnlb, lnrb

    ani = animation.FuncAnimation(fig, update, frames=2, interval=1, blit=True, repeat=True)
    plt.show()

if __name__ == "__main__":
    plot_dyn()
