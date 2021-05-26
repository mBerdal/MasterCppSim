import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
from shapely.ops import cascaded_union
import numpy as np
from matplotlib.patches import Circle

from palette import Pallette
from data_loader import DataLoader

def plot_coverage(data_loader, fig_name=""):
    env_poly = Polygon(data_loader.env_data.corners)
    beacon_poly = Polygon()

    area = []
    for b in data_loader.beacon_data:
        beacon_poly = beacon_poly.union(Point(b.x[-1], b.y[-1]).buffer(data_loader.r_com))
        area.append(beacon_poly.intersection(env_poly).area / env_poly.area)

    fig, ax = plt.subplots()
    ax.plot([i+1 for i in range(data_loader.num_beacons)], area)

    ax.set_xticks(range(1, data_loader.num_beacons+ 1, 2))
    ax.set_xlabel(r"$|\mathcal{B}|$ (number of deployed beacons)")
    ax.set_ylabel(r"$C$ (covered ratio of ROI)")
    ax.grid(True, linestyle="dotted")
    if not fig_name == "":
        fig.savefig(data_loader.get_fig_save_path() + fig_name + "_coverage.pdf", bbox_inches="tight", pad_inches=0)
    plt.show()


def plot_coverages(data_loaders, start_from_beacons=1, fig_name=""):
    assert np.array([data_loaders[i].num_beacons == data_loaders[0].num_beacons for i in range(1, len(data_loaders))]).all()
    assert np.array([data_loaders[i].env_data.name == data_loaders[0].env_data.name for i in range(1, len(data_loaders))]).all()
    
    env_poly = Polygon(data_loaders[0].env_data.corners)
    
    area_mat = np.empty((len(data_loaders), data_loaders[0].num_beacons - start_from_beacons + 1))
    for i in range(len(data_loaders)):
        beacon_poly = cascaded_union([Point(data_loaders[i].beacon_data[b].x[-1], data_loaders[i].beacon_data[b].y[-1]).buffer(data_loaders[i].r_com) for b in range(start_from_beacons - 1)])
        
        for j in range(start_from_beacons - 1, data_loaders[i].num_beacons):
            beacon_poly = beacon_poly.union(
                Point(data_loaders[i].beacon_data[j].x[-1], data_loaders[i].beacon_data[j].y[-1]).buffer(data_loaders[i].r_com)
            )
            area_mat[i, j - start_from_beacons + 1] = beacon_poly.intersection(env_poly).area
    
    area_mat /= env_poly.area
    avg_area = np.average(area_mat, axis=0)
    max_area = np.max(area_mat, axis=0)
    min_area = np.min(area_mat, axis=0)

    assym_errors = [avg_area - min_area, -(avg_area - max_area)]

    fig, ax = plt.subplots()
    ax.errorbar(
        range(start_from_beacons, data_loaders[0].num_beacons + 1),
        avg_area, yerr=assym_errors,
        fmt='o',
        capsize=3,
        ecolor=Pallette.DARK_GRAY,
    )

    ax.set_xticks(range(start_from_beacons, data_loaders[0].num_beacons + 1, 2))
    ax.set_xlabel(r"$|\mathcal{B}|$ (number of deployed beacons)")
    ax.set_ylabel(r"$C$ (covered ratio of ROI)")
    ax.set_ylim([max(0, np.min(min_area) - 0.5), 1.5])
    ax.grid(True, linestyle="dotted")

    if not fig_name == "":
        fig.savefig(data_loaders[0].get_fig_save_path() + fig_name + "_multi_coverages.pdf", bbox_inches="tight", pad_inches=0)
    
    plt.show()


def plot_config(data_loader, fig_name="", plot_coverage=False, include_beacons_limit=None):
    fig, ax = plt.subplots()

    ax.plot(*Polygon(data_loader.env_data.corners).exterior.xy, color=Pallette.DARK_GRAY)

    for b in (data_loader.beacon_data if include_beacons_limit is None else data_loader.beacon_data[:include_beacons_limit]):
        ax.plot(b.x, b.y, linestyle="dashed", alpha=0.4, color=Pallette.LITE_GRAY)
        ax.annotate(b.id, (b.x[-1], b.y[-1]))
        ax.scatter(b.x[-1], b.y[-1], color=Pallette.BLUE, zorder=100)
        ax.plot(
            b.x[-1] + np.array([0, np.cos(b.theta_exp[-1])]),
            b.y[-1] + np.array([0, np.sin(b.theta_exp[-1])]),
            color="black",
            label = r"$\mathbf{v}$" if b.id==0 else None
        )
        if plot_coverage:
            ax.add_patch(Circle((b.x[-1], b.y[-1]), data_loader.r_com, color=Pallette.GREEN, alpha=0.1))

    ax.axis("equal")
    ax.set_xlim([data_loader.env_data.bb_min_x-1, data_loader.env_data.bb_max_x+1])
    ax.set_ylim([data_loader.env_data.bb_min_y-1, data_loader.env_data.bb_max_y+1])
    ax.set_xlabel(r"$x$ [m]")
    ax.set_ylabel(r"$y$ [m]")

    if not fig_name == "":
        fig.savefig(data_loader.get_fig_save_path() + fig_name + "_configuration.pdf", bbox_inches="tight", pad_inches=0)

    plt.legend()
    plt.show()

def plot_uniformity(data_loader, fig_name=""):
    fig, ax = plt.subplots()
    ax.plot(range(1, data_loader.num_beacons + 1), data_loader.uniformity_data)

    ax.set_xticks(range(1, data_loader.num_beacons+ 1, 2))
    ax.set_xlabel(r"$|\mathcal{B}|$ (number of deployed beacons)")
    ax.set_ylabel(r"$U$ (uniformity)")
    ax.grid(True, linestyle="dotted")

    if not fig_name == "":
        fig.savefig(data_loader.get_fig_save_path() + fig_name + "_uniformity.pdf", bbox_inches="tight", pad_inches=0)
    plt.show()

def plot_uniformities(data_loaders, start_from_beacons=1, fig_name=""):
    assert np.array([data_loaders[i].num_beacons == data_loaders[0].num_beacons for i in range(1, len(data_loaders))]).all()
    uniformity_mat = np.array([
        data_loader.uniformity_data[start_from_beacons - 1: ] for data_loader in data_loaders
    ])

    avg_uniformities = np.average(uniformity_mat, axis=0)
    max_uniformities = np.max(uniformity_mat, axis=0)
    min_uniformities = np.min(uniformity_mat, axis=0)
    assym_errors = [avg_uniformities - min_uniformities, -(avg_uniformities - max_uniformities)]

    fig, ax = plt.subplots()
    ax.errorbar(
        range(start_from_beacons, data_loaders[0].num_beacons + 1),
        avg_uniformities, yerr=assym_errors,
        fmt='o',
        capsize=3,
        ecolor=Pallette.DARK_GRAY,
    )

    ax.set_xticks(range(start_from_beacons, data_loaders[0].num_beacons + 1, 2))
    ax.set_xlabel(r"$|\mathcal{B}|$ (number of deployed beacons)")
    ax.set_ylabel(r"$U$ (uniformity)")
    ax.grid(True, linestyle="dotted")

    if not fig_name == "":
        fig.savefig(data_loaders[0].get_fig_save_path() + fig_name + "_multi_uniformities.pdf", bbox_inches="tight", pad_inches=0)

    plt.show()


if __name__ == "__main__":
    xi_param_string = "d_p2,000000_d_n6,000000_xi_b1,000000xi_n_b0,500000"
    data_loaders = [
            DataLoader("uniformity_compare/" + xi_param_string + "/ten_by_ten/50_beacons/run" + str(i) + ".json")
        for i in range(1, 100 + 1)]

    plot_uniformities(
        data_loaders,
        start_from_beacons=10,
        fig_name=xi_param_string
    )

    plot_coverages(data_loaders, start_from_beacons=10, fig_name = xi_param_string)


    d = data_loaders[0]
    plot_coverage(d, xi_param_string)
    for i in range(10, 50 + 1, 10):
        plot_config(d, xi_param_string + "_" + str(i) + "_deployed", plot_coverage=False, include_beacons_limit=i)