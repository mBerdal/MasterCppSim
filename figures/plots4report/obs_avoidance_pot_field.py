import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors


def plot_potential():
    R_s = 4
    v_o_norm =  np.hstack((
        np.linspace(0, R_s - 0.05, 1000),
        np.linspace(R_s - 0.05, R_s, 10000)
    ))

    U = R_s * np.log(R_s / (R_s - v_o_norm)) - v_o_norm

    fig, ax = plt.subplots()
    ax.plot(v_o_norm, U)
    ax.set_xlabel(r"$||\hat{\mathbf{v}}_{o}||$", fontsize=12)
    ax.set_ylabel(r"$U_{o} / \kappa_{o}$", fontsize=12)
    ax.set_xlim([0, R_s + 0.1])
    ax.set_ylim([0, 50])
    ax.set_yticks([0, 20, 40])
    ax.set_xticks([0, R_s / 2, R_s])
    ax.set_xticklabels(["$0$", r"$R_{s} / 2$", r"$R_{s}$"])
    ax.grid("dashed")
    fig.savefig("obs_avoid_potential.pdf")

def plot_force():
    R_s = 4
    n_pts = 32
    radii = np.linspace(0, R_s, 10, endpoint=False)
    radii = np.hstack((radii, [R_s - 0.0000001]))
    thetas = np.linspace(0, 2*np.pi, n_pts, endpoint=False)

    theta, r = np.meshgrid(thetas, radii)
    speed = 1 / (R_s - r)

    f = plt.figure()
    ax = f.add_subplot(111, polar=True)
    ax.set_rticks([R_s]) 
    ax.set_yticklabels([r"$R_{s}$"])
    ax.set_rlim([0, R_s + 1])
    ax.set_rlabel_position(22.5)
    ax.set_xticklabels([
        r"$0$",
        r"$\frac{\pi}{4}$",
        r"$\frac{\pi}{2}$",
        r"$\frac{3 \pi}{4}$",
        r"$\pi$",
        r"$\frac{5 \pi}{4}$",
        r"$\frac{3 \pi}{2}$",
        r"$\frac{7 \pi}{4}$"
    ])
    ax.grid(True)
    Q = ax.quiver(
        theta,
        r,
        np.cos(theta),
        np.sin(theta),
        1 / (R_s - r)
        , pivot='mid', cmap='plasma'
    )
    cb = plt.colorbar(Q, ax=[ax])
    cb.ax.set_ylabel(r"$||\mathbf{F}_{o}|| / \kappa_{o}$", rotation=90)

    f.savefig("obs_avoid_force.pdf")

if __name__ == "__main__":
    plot_force()

    

