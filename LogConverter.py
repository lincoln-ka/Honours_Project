import scipy.io
from pymavlink import mavutil

def bin_to_mat(log_file, output_file):
    mlog = mavutil.mavlink_connection(log_file)
    data = {}

    while True:
        msg = mlog.recv_match()
        if msg is None:
            break
        msg_type = msg.get_type()
        if msg_type == 'BAD_DATA':
            continue
        d = msg.to_dict()
        d.pop('mavpackettype', None)
        if msg_type not in data:
            data[msg_type] = {k: [] for k in d}
        for k, v in d.items():
            data[msg_type][k].append(v)

    # Convert lists to arrays for MATLAB
    import numpy as np
    for msg_type in data:
        for field in data[msg_type]:
            data[msg_type][field] = np.array(data[msg_type][field])

    scipy.io.savemat(output_file, data)

log_file   = r'C:\Users\linky\OneDrive\Documents\Mission Planner\sitl\flightaxis\logs\00000389.BIN'
output_file = r'C:\Users\linky\OneDrive\Documents\2026\IND_PROJ\Honours_Project\Logs\output389.mat'

bin_to_mat(log_file, output_file)
