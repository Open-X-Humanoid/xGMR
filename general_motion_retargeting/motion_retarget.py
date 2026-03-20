from scipy.spatial.transform import Rotation  # type: ignore[import-untyped]
from scipy import sparse as sp  # type: ignore[import-untyped]
import cvxpy as cp  # type: ignore[import-not-found]
import viser  # type: ignore[import-not-found]
import yourdfpy  # type: ignore[import-untyped]
from viser.extras import ViserUrdf  # type: ignore[import-not-found]
import mink
import mujoco as mj
import numpy as np
import json
import time
from scipy.spatial.transform import Rotation as R
from .params import ROBOT_XML_DICT, ROBOT_URDF_DICT, IK_CONFIG_DICT
from rich import print
from general_motion_retargeting.geom_distance_limit import GeomDistanceLimit, FootGroundContactLimit
class GeneralMotionRetargeting:
    """General Motion Retargeting (GMR).
    """
    def __init__(
        self,
        src_human: str,
        tgt_robot: str,
        format: str="",
        actual_human_height: float = None,
        solver: str="daqp", # change from "quadprog" to "daqp".
        damping: float=5e-1, # change from 1e-1 to 1e-2.
        verbose: bool=True,
        use_velocity_limit: bool=False,
        use_collision_avoidance: bool=False,
    ) -> None:
        self.ground_offset = None
        self.horizon_offset = 0.02
        self.lowest_pos = np.inf
        # load the robot model
        if "dex_evt2" in tgt_robot:
            self.xml_file = str(ROBOT_XML_DICT["dex_evt2"])
            self.model_path = str(ROBOT_URDF_DICT["dex_evt2"])
        elif "dex_evt" in tgt_robot:
            self.xml_file = str(ROBOT_XML_DICT["dex_evt"])
            self.model_path = str(ROBOT_URDF_DICT["dex_evt"])
        else:
            self.xml_file = str(ROBOT_XML_DICT[tgt_robot])
            self.model_path = str(ROBOT_URDF_DICT[tgt_robot])
        self.collision_detection_threshold = 0.1
        
        if verbose:
            print("Use robot model: ", self.xml_file)
        self.model = mj.MjModel.from_xml_path(self.xml_file)
        
        # Print DoF names in order
        print("[GMR] Robot Degrees of Freedom (DoF) names and their order:")
        self.robot_dof_names = {}
        for i in range(self.model.nv):  # 'nv' is the number of DoFs
            dof_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_JOINT, self.model.dof_jntid[i])
            self.robot_dof_names[dof_name] = i
            if verbose:
                print(f"DoF {i}: {dof_name}")
            
            
        print("[GMR] Robot Body names and their IDs:")
        self.robot_body_names = {}
        for i in range(self.model.nbody):  # 'nbody' is the number of bodies
            body_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, i)
            self.robot_body_names[body_name] = i
            if verbose:
                print(f"Body ID {i}: {body_name}")
        
        print("[GMR] Robot Motor (Actuator) names and their IDs:")
        self.robot_motor_names = {}
        for i in range(self.model.nu):  # 'nu' is the number of actuators (motors)
            motor_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i)
            self.robot_motor_names[motor_name] = i
            if verbose:
                print(f"Motor ID {i}: {motor_name}")

        # Load the IK config
        if "dex_evt2" in tgt_robot:
            config_path = IK_CONFIG_DICT[src_human]["dex_evt_" + format]
        else:
            config_path = IK_CONFIG_DICT[src_human][tgt_robot + "_" + format]
        

        with open(config_path) as f:
            ik_config = json.load(f)
        if verbose:
            print("Use IK config: ", config_path)
        
        if actual_human_height is not None:
            ratio = actual_human_height / ik_config["human_height_assumption"]
        else:
            ratio = 1.0

         # adjust the human scale table
        for key in ik_config["human_scale_table"].keys():
            ik_config["human_scale_table"][key] = ik_config["human_scale_table"][key] * ratio

        # used for retargeting
        self.ik_match_table1 = ik_config["ik_match_table1"]
        self.ik_match_table2 = ik_config["ik_match_table2"]
        self.human_root_name = ik_config["human_root_name"]
        self.robot_root_name = ik_config["robot_root_name"]
        self.use_ik_match_table1 = ik_config["use_ik_match_table1"]
        self.use_ik_match_table2 = ik_config["use_ik_match_table2"]
        self.human_scale_table = ik_config["human_scale_table"]
        self.ground = ik_config["ground_height"] * np.array([0, 0, 1])

        self.max_iter = 10

        self.solver = solver
        self.damping = damping

        self.human_body_to_task1 = {}
        self.human_body_to_task2 = {}
        self.pos_offsets1 = {}
        self.rot_offsets1 = {}
        self.pos_offsets2 = {}
        self.rot_offsets2 = {}

        self.task_errors1 = {}
        self.task_errors2 = {}

        self.ik_limits = [mink.ConfigurationLimit(self.model)]
        if use_velocity_limit:
            # Velocity limits should be applied on non-free joints, not actuators.
            # Velocity limits should be applied on non-free joints, not actuators.
            free_joints = set(
                mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_JOINT, j)
                for j in range(self.model.njnt)
                if self.model.jnt_type[j] == mj.mjtJoint.mjJNT_FREE
            )
            # Only constrain joints explicitly listed in config.
            velocity_limits = {}
            config_velocity_limits = ik_config.get("robot_velocity_limits", {})
            for joint_name, limit in config_velocity_limits.items():
                # Allow both joint names and motor (actuator) names in config.
                if joint_name in self.robot_dof_names:
                    if joint_name in free_joints:
                        print(f"[GMR][warn] skip velocity limit on free joint '{joint_name}'")
                        continue
                    velocity_limits[joint_name] = limit
                elif joint_name in self.robot_motor_names:
                    motor_id = self.robot_motor_names[joint_name]
                    # actuator_trnid gives (joint id, type); use joint id to find the driven joint.
                    driven_joint_id = self.model.actuator_trnid[motor_id, 0]
                    driven_joint_name = mj.mj_id2name(
                        self.model, mj.mjtObj.mjOBJ_JOINT, driven_joint_id
                    )
                    if driven_joint_name in free_joints:
                        print(f"[GMR][warn] skip velocity limit on free joint driven by motor '{joint_name}'")
                        continue
                    velocity_limits[driven_joint_name] = limit
                else:
                    print(f"[GMR][warn] skipping velocity limit for unsupported or unknown joint/motor '{joint_name}'")
            if velocity_limits:
                self.ik_limits.append(mink.VelocityLimit(self.model, velocity_limits))
            else:
                print("[GMR][warn] no velocity limits applied (config empty or all skipped)")

        ground_geom = None
        for candidate in ("ground", "floor"):
            if mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, candidate) >= 0:
                ground_geom = candidate
                break
        foot_geoms = [
            "wrist_pitch_l_link_collision",
            "wrist_pitch_r_link_collision",
            "wrist_roll_l_link_collision",
            "wrist_roll_r_link_collision"
            "pelvis_collision",
            "knee_pitch_l_link_collision",
            "knee_pitch_r_link_collision",
        ]
        foot_geoms = [
            name
            for name in foot_geoms
            if mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, name) >= 0
        ]
        if ground_geom and foot_geoms:
            self.foot_ground_limit = FootGroundContactLimit(
                model=self.model,
                foot_geom_names=foot_geoms,
                min_height=0.025,
                margin=0.01,
            )
            self.ik_limits.append(self.foot_ground_limit)
            
            if verbose:
                print(
                    f"[GMR] foot-ground collision avoidance enabled: {foot_geoms} vs {ground_geom}"
                )
        elif verbose:
            print(
                "[GMR][warn] foot-ground collision avoidance skipped (missing ground or foot geoms)"
            )
        
        foot_geoms = [
            "toe1_left",
            "toe1_right",
            "toe2_left",
            "toe2_right",
            "toe_r_link_collision",
            "toe_l_link_collision",
            "foot_end_l_link_collision",
            "foot_end_r_link_collision",
            "hip_yaw_l_link_collision",
            "hip_yaw_r_link_collision"
            "ankle_roll_l_link_collision",
            "ankle_roll_r_link_collision",
            "left_tcp_link_collision",
            "right_tcp_link_collision",
        ]
        foot_geoms = [
            name
            for name in foot_geoms
            if mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, name) >= 0
        ]
        if ground_geom and foot_geoms:
            self.foot_ground_limit = FootGroundContactLimit(
                model=self.model,
                foot_geom_names=foot_geoms,
                min_height=self.horizon_offset,
            )
            print(f"Foot ground limit created with foot geoms: {foot_geoms}")
            self.ik_limits.append(self.foot_ground_limit)
                
        self.last_cost = np.inf

        if use_collision_avoidance:
            self.col_limit1 = GeomDistanceLimit(
                model=self.model,
                geom_pairs=[
                        (["head_yaw_link_collision", "wrist_pitch_l_link_collision"]),
                        (["head_yaw_link_collision", "wrist_pitch_r_link_collision"]),
                        (["head_pitch_link_collision", "wrist_pitch_l_link_collision"]),
                        (["head_pitch_link_collision", "wrist_pitch_r_link_collision"]),
                        (["wrist_roll_l_link_collision"], ["hip_roll_l_link_collision"]),
                        (["wrist_roll_l_link_collision"], ["shoulder_yaw_l_link_collision"]),
                        (["wrist_roll_r_link_collision"], ["hip_roll_r_link_collision"]),  
                        (["wrist_roll_r_link_collision"], ["shoulder_roll_r_link_collision"]),
                        (["wrist_roll_r_link_collision"], ["wrist_roll_l_link_collision"]),
                        (["hip_yaw_l_link_collision", "left_tcp_link_collision"]),
                        (["hip_yaw_r_link_collision", "right_tcp_link_collision"]),
                        (["hip_roll_l_link_collision", "left_tcp_link_collision"]),
                        (["hip_roll_r_link_collision", "right_tcp_link_collision"]),
                        (["elbow_pitch_r_link_collision", "left_tcp_link_collision"]),
                        (["wrist_roll_r_link_collision", "left_tcp_link_collision"]),
                        (["elbow_pitch_l_link_collision", "right_tcp_link_collision"]),
                        (["wrist_roll_l_link_collision", "right_tcp_link_collision"]),
                        (["left_tcp_link_collision", "right_tcp_link_collision"]),
                        (["knee_pitch_l_link_collision", "left_tcp_link_collision"]),
                        (["knee_pitch_r_link_collision", "right_tcp_link_collision"]),
                        (["hip_yaw_l_link_collision", "wrist_pitch_l_link_collision"]),
                        (["hip_yaw_r_link_collision", "wrist_pitch_r_link_collision"]),
                        (["hip_roll_l_link_collision", "wrist_pitch_l_link_collision"]),
                        (["hip_roll_r_link_collision", "wrist_pitch_r_link_collision"]),
                        (["hip_yaw_l_link_collision", "wrist_roll_l_link_collision"]),
                        (["hip_yaw_r_link_collision", "wrist_roll_r_link_collision"]),
                        (["hip_roll_l_link_collision", "wrist_roll_l_link_collision"]),
                        (["hip_roll_r_link_collision", "wrist_roll_r_link_collision"]),
                        (["hip_yaw_l_link_collision", "elbow_pitch_l_link_collision"]),
                        (["hip_yaw_r_link_collision", "elbow_pitch_r_link_collision"]),
                        (["hip_roll_l_link_collision", "elbow_pitch_l_link_collision"]),
                        (["hip_roll_r_link_collision", "elbow_pitch_r_link_collision"]),
                        (["hip_yaw_l_link_collision", "elbow_yaw_l_link_collision"]),
                        (["hip_yaw_r_link_collision", "elbow_yaw_r_link_collision"]),
                        (["hip_roll_l_link_collision", "elbow_yaw_l_link_collision"]),
                        (["hip_roll_r_link_collision", "elbow_yaw_r_link_collision"]),
                        (["wrist_roll_l_link_collision"], ["knee_pitch_l_link_collision"]),
                        (["wrist_roll_r_link_collision"], ["knee_pitch_r_link_collision"]),
                        (["elbow_pitch_l_link_collision"], ["knee_pitch_l_link_collision"]),
                        (["elbow_pitch_r_link_collision"], ["knee_pitch_r_link_collision"]),
                        ],
                gain=1e-3,
                minimum_distance_from_collisions=0.015,      
                collision_detection_distance=0.035,         
            )
            self.ik_limits.append(self.col_limit1)
            print("col_limit1 has been started")
            self.col_limit2 = GeomDistanceLimit(
                model=self.model,
                geom_pairs=[
                        (["wrist_roll_l_link_collision"], ["pelvis_collision"]),
                        (["wrist_roll_r_link_collision"], ["pelvis_collision"]),
                        (["left_tcp_link_collision"], ["pelvis_collision"]),
                        (["right_tcp_link_collision"], ["pelvis_collision"]),
                        (["hip_yaw_r_link_collision"], ["hip_yaw_l_link_collision"]),
                        (["knee_pitch_l_link_collision"], ["knee_pitch_r_link_collision"]),
                        (["knee_pitch_l_link_collision"], ["hip_yaw_r_link_collision"]),
                        (["knee_pitch_r_link_collision"], ["hip_yaw_l_link_collision"]),
                        
                        (["elbow_pitch_r_link_collision", "elbow_pitch_l_link_collision"]),
                        (["wrist_roll_r_link_collision", "elbow_pitch_l_link_collision"]),
                        (["elbow_pitch_r_link_collision", "elbow_yaw_l_link_collision"]),
                        (["wrist_roll_r_link_collision", "elbow_yaw_l_link_collision"]),

                        (["elbow_pitch_l_link_collision", "elbow_pitch_r_link_collision"]),
                        (["wrist_roll_l_link_collision", "elbow_pitch_r_link_collision"]),
                        (["elbow_pitch_l_link_collision", "elbow_yaw_r_link_collision"]),
                        (["wrist_roll_l_link_collision", "elbow_yaw_r_link_collision"]),
                        ],
                gain=1e-3,
                minimum_distance_from_collisions=0.005,      
                collision_detection_distance=0.025,         
                bound_relaxation=0,
            )
            self.ik_limits.append(self.col_limit2)
            
        self.setup_retarget_configuration()

        self.toe_id = [mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "toe_l_link"), mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "toe_r_link"), mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "pelvis")]


    def setup_retarget_configuration(self):
        self.configuration = mink.Configuration(self.model)
    # 任务2重点是腿，任务1是上半身
        self.tasks1 = []
        self.tasks2 = []
        
        for frame_name, entry in self.ik_match_table1.items():
            # import ipdb; ipdb.set_trace()
            # breakpoint()
            body_name, pos_weight, rot_weight, pos_offset, rot_offset = entry
            # print(f"body_name is {body_name} , pos_weight is {pos_weight} , rot_weight is {rot_weight} , pos_offset is {pos_offset} , rot_offset is {rot_offset}")
            if pos_weight != 0 or rot_weight != 0:
                task = mink.FrameTask(
                    frame_name=frame_name,
                    frame_type="body",
                    position_cost=pos_weight,
                    orientation_cost=rot_weight,
                    lm_damping=1,
                )
                self.human_body_to_task1[body_name] = task
                self.pos_offsets1[body_name] = np.array(pos_offset) - self.ground
                self.rot_offsets1[body_name] = R.from_quat(
                    rot_offset, scalar_first=True
                )
                self.tasks1.append(task)
                self.task_errors1[task] = []

        for frame_name, entry in self.ik_match_table2.items():
            body_name, pos_weight, rot_weight, pos_offset, rot_offset = entry
            # print(f"body_name is {body_name} , pos_weight is {pos_weight} , rot_weight is {rot_weight} , pos_offset is {pos_offset} , rot_offset is {rot_offset}")
            if pos_weight != 0 or rot_weight != 0:
                task = mink.FrameTask(
                    frame_name=frame_name,
                    frame_type="body",
                    position_cost=pos_weight,
                    orientation_cost=rot_weight,
                    lm_damping=1,
                )
                self.human_body_to_task2[body_name] = task
                self.pos_offsets2[body_name] = np.array(pos_offset) - self.ground
                self.rot_offsets2[body_name] = R.from_quat(
                    rot_offset, scalar_first=True
                )
                self.tasks2.append(task)
                self.task_errors2[task] = []
  
    def update_targets(self, human_data, offset_to_ground=False):
        # scale human data in local frame
        human_data = self.to_numpy(human_data)
        human_data = self.scale_human_data(human_data, self.human_root_name, self.human_scale_table)
        human_data = self.offset_human_data(human_data, self.pos_offsets1, self.rot_offsets1)

        human_data = self.apply_ground_offset(human_data)
        
        self.scaled_human_data = human_data
    
        if self.use_ik_match_table1:
            for body_name in self.human_body_to_task1.keys():
                task = self.human_body_to_task1[body_name]
                pos, rot = human_data[body_name]
                task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos))
        
        if self.use_ik_match_table2:
            for body_name in self.human_body_to_task2.keys():
                task = self.human_body_to_task2[body_name]
                pos, rot = human_data[body_name]
                task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos))
    
    def _prefilter_pairs_with_mj_collision(self, threshold: float):
        m, d = self.model, self.data
        ngeom = m.ngeom

        self._geom_names = [mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, g) or "" for g in range(ngeom)]

        if not hasattr(self, "_saved_margins"):
            self._saved_margins = np.empty_like(m.geom_margin)
        self._saved_margins[:] = m.geom_margin

        m.geom_margin[:] = threshold

        # Run collision. This runs broad→narrow and fills d.contact.
        mj.mj_collision(m, d)

        # Collect unique candidate pairs that involve at least one masked geom
        candidates = set()
        for k in range(d.ncon):
            c = d.contact[k]
            g1, g2 = int(c.geom1), int(c.geom2)
            if g1 < 0 or g2 < 0:
                continue
            candidates.add((min(g1, g2), max(g1, g2)))

        # Restore margins to keep physics untouched
        m.geom_margin[:] = self._saved_margins

        return candidates

    def retarget(self, human_data, offset_to_ground=False, target_laplacian=None, adj_list=None, object_points_local=None, w_nominal_tracking=5.0, q_nominal_list=None, i=0, t=0):
        # Update the task targets
        self.update_targets(human_data, offset_to_ground)

        if self.use_ik_match_table1:
            # Solve the IK problem
            curr_error = self.error1()
            dt = self.configuration.model.opt.timestep
            vel1 = mink.solve_ik(
                self.configuration, self.tasks1, dt, self.solver, self.damping, self.ik_limits
            )
            self.configuration.integrate_inplace(vel1, dt)
            next_error = self.error1()
            num_iter = 0
            while curr_error - next_error > 0.001 and num_iter < self.max_iter:
                curr_error = next_error
                dt = self.configuration.model.opt.timestep
                vel1 = mink.solve_ik(
                    self.configuration, self.tasks1, dt, self.solver, self.damping, self.ik_limits
                )
                self.configuration.integrate_inplace(vel1, dt)
                next_error = self.error1()
                num_iter += 1

        if self.use_ik_match_table2:
            curr_error = self.error2()
            dt = self.configuration.model.opt.timestep
            vel2 = mink.solve_ik(
                self.configuration, self.tasks2, dt, self.solver, self.damping, self.ik_limits
            )
            self.configuration.integrate_inplace(vel2, dt)
            next_error = self.error2()
            num_iter = 0
            while curr_error - next_error > 0.001 and num_iter < self.max_iter:
                curr_error = next_error
                # Solve the IK problem with the second task
                dt = self.configuration.model.opt.timestep
                vel2 = mink.solve_ik(
                    self.configuration, self.tasks2, dt, self.solver, self.damping, self.ik_limits
                )
                self.configuration.integrate_inplace(vel2, dt)
                
                next_error = self.error2()
                num_iter += 1
            
        return self.configuration.data.qpos.copy()


    def error1(self):
        all_errors = []
        for task in self.tasks1:
            err = task.compute_error(self.configuration)
            # 如果这个 task 是 GroundAvoidanceTask，就打印
            # if isinstance(task, GroundAvoidanceTask):
            #     print(f"[Ground Task] {task.body_name} error: {err}")
            all_errors.append(err)
        
        all_errors = np.concatenate(all_errors)
        return np.linalg.norm(all_errors)
    
    def error2(self):
        all_errors = []
        for task in self.tasks2:
            err = task.compute_error(self.configuration)
            # 如果这个 task 是 GroundAvoidanceTask，就打印
            # if isinstance(task, GroundAvoidanceTask):
            #     print(f"[Ground Task] {task.body_name} error: {err}")
            all_errors.append(err)
        
        all_errors = np.concatenate(all_errors)
        return np.linalg.norm(all_errors)


    def to_numpy(self, human_data):
        for body_name in human_data.keys():
            human_data[body_name] = [np.asarray(human_data[body_name][0]), np.asarray(human_data[body_name][1])]
        return human_data


    def scale_human_data(self, human_data, human_root_name, human_scale_table):
        
        human_data_local = {}
        root_pos, root_quat = human_data[human_root_name]
        # scale root
        scaled_root_pos = human_scale_table[human_root_name] * root_pos
        
        # scale other body parts in local frame
        for body_name in human_data.keys():
            if body_name not in human_scale_table:
                continue
            if body_name == human_root_name:
                continue
            else:
                # transform to local frame (only position)
                human_data_local[body_name] = (human_data[body_name][0] - root_pos) * human_scale_table[body_name]
            
        # transform the human data back to the global frame
        human_data_global = {human_root_name: (scaled_root_pos, root_quat)}
        for body_name in human_data_local.keys():
            human_data_global[body_name] = (human_data_local[body_name] + scaled_root_pos, human_data[body_name][1])

        return human_data_global
    
    def offset_human_data(self, human_data, pos_offsets, rot_offsets):
        """the pos offsets are applied in the local frame"""
        offset_human_data = {}
        for body_name in human_data.keys():
            pos, quat = human_data[body_name]
            offset_human_data[body_name] = [pos, quat]
            # apply rotation offset first
            updated_quat = (R.from_quat(quat, scalar_first=True) * rot_offsets[body_name]).as_quat(scalar_first=True)
            offset_human_data[body_name][1] = updated_quat
            
            local_offset = pos_offsets[body_name]
            # compute the global position offset using the updated rotation
            global_pos_offset = R.from_quat(updated_quat, scalar_first=True).apply(local_offset)
            
            offset_human_data[body_name][0] = pos + global_pos_offset
           
        return offset_human_data
    
    

    def set_ground_offset(self, ground_offset):
        self.ground_offset = ground_offset

    def apply_ground_offset(self, human_data):
        toe_rela_pos = min([float(self.configuration.data.xpos[self.toe_id[0]][2]), float(self.configuration.data.xpos[self.toe_id[1]][2])])
        cur_lowest_pos = np.inf
        lowest_body_name = ""
            
        for body_name, (pos, quat) in human_data.items():
            if pos[2] < cur_lowest_pos:
                cur_lowest_pos = pos[2]
                lowest_body_name = body_name

        if "Foot" in lowest_body_name or "Toe" in lowest_body_name:
            if toe_rela_pos < 0:
                cur_lowest_pos += toe_rela_pos

        if self.ground_offset is None:
            self.cur_lowest_pos = cur_lowest_pos
            self.ground_offset =  cur_lowest_pos - self.horizon_offset
        elif "Wrist" in lowest_body_name or "Hand" in lowest_body_name:
            ground_offset =  cur_lowest_pos - self.horizon_offset

        if cur_lowest_pos - self.horizon_offset < self.ground_offset:
            ground_offset =  cur_lowest_pos - self.horizon_offset
        else:
            ground_offset = self.ground_offset

        for body_name in human_data.keys():
            pos, quat = human_data[body_name]
            human_data[body_name][0] = pos - np.array([0, 0, ground_offset])        

        return human_data
