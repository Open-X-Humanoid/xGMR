import argparse
import pathlib
import time
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.lafan1 import load_bvh_file
from rich import print
from tqdm import tqdm
import os
import numpy as np

if __name__ == "__main__":
    
    HERE = pathlib.Path(__file__).parent

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bvh_file",
        help="BVH motion file to load.",
        required=True,
        type=str,
    )
    
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands", "booster_t1", "stanford_toddy", "fourier_n1", 
            "engineai_pm01","dex0902","dex_v3", "dex_evt", "tienkung2", "tienkung2_lite", "tienkung2_plus", "dex_evt2"],
        default="unitree_g1",
    )
        
    parser.add_argument(
        "--record_video",
        action="store_true",
        default=False,
    )

    parser.add_argument(
        "--video_path",
        type=str,
        default="videos/example.mp4",
    )

    parser.add_argument(
        "--rate_limit",
        action="store_true",
        default=False,
    )

    parser.add_argument(
        "--save_path",
        default=None,
        help="Path to save the robot motion.",
    )
    
    def str2bool(v):
        return v.lower() in ("true", "1", "yes")

    parser.add_argument(
        "--add_collision_avoidance",
        type=str2bool,
        default=False,
    )

    parser.add_argument(
        "--add_offset_human_data",
        type=str2bool,
        default=False,
        
    )

    parser.add_argument(
        "--format",
        type=str,
        default="",
    )

    parser.add_argument(
        "--motion_fps",
        type=int,
        default=100,
    )

    parser.add_argument(
        "--object_mesh_file",
        type=str,
        default="assets/dex_evt/meshes_new/box_obstacle_xup.stl",
        
    )

    parser.add_argument(
        "--task_type",
        type=str,
        default="robot_only",
        
    )

    args = parser.parse_args()
    

    if args.save_path is not None:
        args.save_path = args.save_path[:-4] + args.robot + args.save_path[-4:]
        save_dir = os.path.dirname(args.save_path)
        if save_dir:  # Only create directory if it's not empty
            os.makedirs(save_dir, exist_ok=True)
        qpos_list = []
        dof_vels = []
        end_poses = []

    # Initialize the retargeting system
    lafan1_data_frames, actual_human_height = load_bvh_file(args.bvh_file, format=args.format)
    
    retargeter = GMR(
        src_human="bvh",
        tgt_robot=args.robot,
        format=args.format,
        actual_human_height=actual_human_height,
        use_collision_avoidance=args.add_collision_avoidance,
    )
    
    robot_motion_viewer = RobotMotionViewer(robot_type=args.robot,
                                            motion_fps=args.motion_fps,
                                            transparent_robot=0,
                                            record_video=args.record_video,
                                            video_path=args.video_path,
                                            )
    
    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds
    
    print(f"mocap_frame_rate: {args.motion_fps}")
    
    # Create tqdm progress bar for the total number of frames
    pbar = tqdm(total=len(lafan1_data_frames), desc="Retargeting")
    
    # Start the viewer
    i = 0
    pre_dof_pos = None

    while i < len(lafan1_data_frames):
        
        # FPS measurement
        fps_counter += 1
        current_time = time.time()
        if current_time - fps_start_time >= fps_display_interval:
            actual_fps = fps_counter / (current_time - fps_start_time)
            print(f"Actual rendering FPS: {actual_fps:.2f}")
            fps_counter = 0
            fps_start_time = current_time
            
        # Update progress bar
        pbar.update(1)

        # Update task targets.
        smplx_data = lafan1_data_frames[i]

        # retarget
        qpos = retargeter.retarget(smplx_data, args.add_offset_human_data)
        if i > 1: # 前几帧可能有较大抖动，导致速度过大，先不保存
            qpos_list.append(qpos)
            end_poses.append(end_pos.tolist())
            dof_vels.append(dof_vel.tolist())
        else: 
            retargeter.ground_offset = None
            retargeter.configuration.data.qpos[:3] == lafan1_data_frames[i]["Hips"][0].copy()
            retargeter.configuration.data.qpos[3:7] == lafan1_data_frames[i]["Hips"][1].copy()
            qpos = retargeter.configuration.data.qpos.copy()
        # visualize
        robot_motion_viewer.step(
            root_pos=qpos[:3],
            root_rot=qpos[3:7],
            dof_pos=qpos[7:],
            human_motion_data=retargeter.scaled_human_data,
            rate_limit=args.rate_limit,
        )
        end_pos = robot_motion_viewer.get_info()
        dof_pos = qpos[7:]
        if pre_dof_pos is None:
            dof_vel = np.zeros(dof_pos.shape)
            pre_dof_pos = qpos.copy()
        else:
            dof_vel = (qpos - pre_dof_pos) / (1 / args.motion_fps)

        i += 1
        

    if args.save_path is not None:
        import pickle
        root_pos = np.array([qpos[:3] for qpos in qpos_list])
        # save from wxyz to xyzw
        root_rot = np.array([qpos[3:7][[1,2,3,0]] for qpos in qpos_list])
        dof_pos = np.array([qpos[7:] for qpos in qpos_list])
        local_body_pos = None
        body_names = None
        
        motion_data = {
            "fps": args.motion_fps,
            "root_pos": root_pos,
            "root_rot": root_rot,
            "dof_pos": dof_pos,
            "local_body_pos": local_body_pos,
            "link_body_list": body_names,
        }
        
        if args.save_path[-3:] == "pkl":
            with open(args.save_path, "wb") as f:
                pickle.dump(motion_data, f)
        elif args.save_path[-3:] == "npz":
            motion_data["dof_vel"] = np.array(dof_vels)
            motion_data["endpoint_pos_BCS"] = np.array(end_poses)
            np.savez(args.save_path, **motion_data)
        print(f"Saved to {args.save_path}")

    # Close progress bar
    pbar.close()
    
    robot_motion_viewer.close()
       
