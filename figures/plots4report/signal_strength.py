import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    d_perf = 3
    d_none = 10
    xi_bar = 20

    line_clr = "#326fa8"
    line_width = 2

    fig, ax = plt.subplots()

    ax.plot([0, d_perf], [xi_bar, xi_bar], color=line_clr, linewidth=line_width)

    d = np.linspace(d_perf, d_none, 1000)
    xi = (xi_bar / 2) * (1 + np.cos(np.pi * (d - d_perf) / (d_none - d_perf)))
    ax.plot(d, xi, color=line_clr, linewidth=line_width)

    ax.plot([d_none, d_none + 3], [0, 0], color=line_clr, linewidth=line_width)
    ax.set_yticks([0, xi_bar])
    ax.set_yticklabels(['0', r'$\bar{\xi}$'])

    ax.set_xticks([d_perf, d_none])
    ax.set_xticklabels([r'$d_{perf}$', r'$d_{none}$'])

    ax.set_xlabel(r'$d_{ij}$')
    ax.set_ylabel(r'$\xi_{ij}$')
    ax.grid(ls="dashed")


    fig.savefig("signal_strength.pdf")
    plt.show()

