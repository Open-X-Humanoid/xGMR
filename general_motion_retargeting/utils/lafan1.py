import numpy as np
from scipy.spatial.transform import Rotation as R

import general_motion_retargeting.utils.lafan_vendor.utils as utils
from general_motion_retargeting.utils.lafan_vendor.extract import read_bvh


def load_bvh_file(bvh_file, format="xsens"):
    """
    Must return a dictionary with the following structure:
    {
        "Hips": (position, orientation),
        "Spine": (position, orientation),
        ...
    }
    """
    data = read_bvh(bvh_file)
    global_data = utils.quat_fk(data.quats, data.pos, data.parents)

    rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    rotation_quat = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)

    frames = []
    human_joints = []
    lowest_foot = np.inf
    lowest_foot_frame = None

    for frame in range(data.pos.shape[0]):
        result = {}
        for i, bone in enumerate(data.bones):
            orientation = utils.quat_mul(rotation_quat, global_data[0][frame, i])
            position = global_data[1][frame, i] @ rotation_matrix.T / 100  # cm to m
            result[bone] = [position, orientation]

        if format == "lafan":
            # Add modified foot pose
            result["LeftFootMod"] = [result["LeftFoot"][0], result["LeftToe"][1]]
            result["RightFootMod"] = [result["RightFoot"][0], result["RightToe"][1]]
        elif format == "nokov":
            result["LeftFootMod"] = [result["LeftFoot"][0], result["LeftToeBase"][1]]
            result["RightFootMod"] = [result["RightFoot"][0], result["RightToeBase"][1]]
        elif format == "xsens":
            result["LeftFootMod"] = (result["LeftAnkle"][0], result["LeftToe"][1])
            result["RightFootMod"] = (result["RightAnkle"][0], result["RightToe"][1])
        else:
            try:
                result["LeftFootMod"] = (result["LeftFoot"][0], result["LeftToe"][1])
                result["RightFootMod"] = (result["RightFoot"][0], result["RightToe"][1])
            except Exception as e:
                try:
                    result["LeftFootMod"] = (result["LeftFoot"][0], result["LeftToeBase"][1])
                    result["RightFootMod"] = (result["RightFoot"][0], result["RightToeBase"][1])
                except Exception as e:
                    result["LeftFootMod"] = result["LeftFoot"]
                    result["RightFootMod"] = result["RightFoot"]
        frames.append(result)
    human_height = frames[0]["Head"][0][2] - min(frames[0]["LeftFootMod"][0][2], frames[0]["RightFootMod"][0][2])
    # human_height = human_height + 0.2  # cm to m
    return frames, human_height

