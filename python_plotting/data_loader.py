import json
import numpy as np

class SingleBeaconData:
    def __init__(self, data_dict, id):
        self.x = data_dict["x"]
        self.y = data_dict["y"]
        self.v_x = data_dict["v_x"]
        self.v_y = data_dict["v_y"]
        self.yaw = data_dict["yaw"]
        self.f_e_x = data_dict["f_e_x"]
        self.f_e_y = data_dict["f_e_y"]
        self.f_n_x = data_dict["f_n_x"]
        self.f_n_y = data_dict["f_n_y"]
        self.f_x = data_dict["f_x"]
        self.f_y = data_dict["f_y"]
        self.theta_exp = data_dict["theta_exp"]
        self.time = data_dict["time"]
        self.id = id

class EnvData:
    def __init__(self, env_data):
        self.bb_min_x = env_data["bounding_box"]["min_x"]
        self.bb_max_x = env_data["bounding_box"]["max_x"]
        self.bb_min_y = env_data["bounding_box"]["min_y"]
        self.bb_max_y = env_data["bounding_box"]["max_y"]
        self.name = env_data["name"]
        self.corners = env_data["walls"]

class XiData:
    def __init__(self, xi_data):
        self.n_a_thresh = xi_data["agent_neigh_threshold"]
        self.n_b_thresh = xi_data["beacon_neigh_threshold"]
        self.d_none = xi_data["d_none"]
        self.d_perf = xi_data["d_perf"]
        self.xi_bar = xi_data["xi_bar"]

class DataLoader:
    DATA_PATH = "../data/"
    FIGURE_PATH = "../figures/"

    def __init__(self, path):
        self.beacon_data = []
        with open(DataLoader.DATA_PATH + path) as f:
            data = json.load(f)
            self.num_beacons = len(data["beacon_data"])

            for i in range(self.num_beacons):
                self.beacon_data.append(SingleBeaconData(data["beacon_data"][i], i))
            
            self.env_data = EnvData(data["environment"])
            self.xi_data = XiData(data["signal_strength_data"])
            self.uniformity_data = data["uniformity"]
        
        self.r_com = ((self.xi_data.d_none - self.xi_data.d_perf) / np.pi) *\
            np.arccos((2 * self.xi_data.n_b_thresh - self.xi_data.xi_bar) / self.xi_data.xi_bar) + self.xi_data.d_perf

    def get_fig_save_path(self):
        return DataLoader.FIGURE_PATH + self.env_data.name + "/" + str(self.num_beacons) + "_beacons/"

    