import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors

def U_att(X, Y):
    return (1/2) * np.sqrt(X**2 + Y**2)**2

def U_rep(X, Y):
    return (1/2) * (1 / np.sqrt(X**2 + Y**2))**2

def plot_field(X, Y, U, pot_name):
    fig = plt.figure(figsize=(13, 5))
    u_ax = fig.add_subplot(121, projection="3d")
    f_ax = fig.add_subplot(122)

    v_y, v_x = np.gradient(-U, .2, .2)
    speed = np.sqrt(v_x**2 + v_y**2)

    Q = f_ax.quiver(
        X,
        Y,
        v_x/speed,
        v_y/speed,
        speed,
        cmap="plasma",
        norm=colors.LogNorm(vmin=speed.min(),vmax=speed.max())
    )
    cb = plt.colorbar(Q, ax=[f_ax])
    cb.ax.set_ylabel(r"$||\mathbf{F}||$", rotation=90)

    f_ax.set_title(r"$\mathbf{F}_{" + pot_name + r"}$")
    f_ax.set_xlabel("$x - x_{0}$")
    f_ax.set_ylabel("$y - y_{0}$")
    f_ax.set_xticks([X.min(), 0, X.max()])
    f_ax.set_yticks([Y.min(), 0, Y.max()])

    u_ax.plot_surface(X, Y, U, cmap="plasma")
    u_ax.set_title(r"$U_{" + pot_name + r"}$")
    u_ax.set_xlabel("$x - x_{0}$")
    u_ax.set_ylabel("$y - y_{0}$")
    u_ax.set_zlabel("U")
    u_ax.set_xticks([X.min(), 0, X.max()])
    u_ax.set_yticks([Y.min(), 0, Y.max()])
    fig.savefig("pot_field_" + pot_name + ".pdf")


if __name__ == "__main__":
    X, Y = np.meshgrid(
        np.linspace(-10,10,20),
        np.linspace(-10,10,20)
    )
    U = U_rep(X, Y)
    plot_field(X, Y, U, "rep")
