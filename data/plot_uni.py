import json
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    with open("uniformity_test_v2.json") as f:
        """
        data formated as dict where keys correspond to the number
        of beacons deployed, and values are arrays of lenght num_runs_per_iteration containing
        the uniformity after deploying all beacons in different runs
        """
        data = json.load(f)
        initial_num_deployed_beacons = int(list(data.keys())[0])

        num_iterations = len(data)
        num_runs_per_iteration = len(data[str(initial_num_deployed_beacons)])

        num_deployed_beacons_array = np.arange(num_iterations) + initial_num_deployed_beacons

        # Load data into numpy matrix
        data_mat = np.empty((num_iterations, num_runs_per_iteration))
        for num_deployed_beacons, uniformity_array in data.items():
                data_mat[int(num_deployed_beacons) - initial_num_deployed_beacons, :] = np.array(uniformity_array)

    mean_uniformities = np.mean(data_mat, axis=1)
    lb = mean_uniformities - np.min(data_mat, axis=1)
    ub = np.max(data_mat, axis=1) - mean_uniformities

    assym_errors = [lb, ub]
    fig, ax = plt.subplots()
    ax.errorbar(num_deployed_beacons_array, mean_uniformities, yerr=assym_errors, fmt='o', capsize=3)
    ax.set_ylim([0, 1.6])

    ax.grid(linestyle='dotted')
    ax.set_xlabel("Total number of beacons")
    ax.set_ylabel("Uniformity")
    plt.show()