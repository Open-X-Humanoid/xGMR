import mujoco as mj
import numpy as np

from mink import Constraint
from mink import Limit


class GeomDistanceLimit(Limit):
    """Collision avoidance using mj_geomDistance between geoms."""

    def __init__(
        self,
        model,
        geom_pairs,
        gain=0.2,
        minimum_distance_from_collisions=0.02,
        collision_detection_distance=0.12,
        bound_relaxation=0.0,
    ):
        self.model = model
        self.gain = gain
        self.minimum_distance_from_collisions = minimum_distance_from_collisions
        self.collision_detection_distance = collision_detection_distance
        self.bound_relaxation = bound_relaxation
        self.geom_pairs = self._resolve_geom_pairs(geom_pairs)

    def _resolve_geom_pairs(self, geom_pairs):
        resolved = []
        for group_a, group_b in geom_pairs:
            ids_a = [self._geom_id(name) for name in self._ensure_list(group_a)]
            ids_b = [self._geom_id(name) for name in self._ensure_list(group_b)]
            resolved.append((ids_a, ids_b, self._ensure_list(group_a), self._ensure_list(group_b)))
        return resolved

    @staticmethod
    def _ensure_list(group):
        if isinstance(group, str):
            return [group]
        return list(group)

    def _geom_id(self, name):
        geom_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            geom_names = self._list_geom_names()
            print(f"[GMR][geom] available geoms ({len(geom_names)}): {geom_names}")
            raise ValueError(f"[GMR] geom '{name}' not found in model")
        return geom_id

    def _list_geom_names(self):
        """Return all geom names (for debugging)."""
        names = []
        for i in range(self.model.ngeom):
            nm = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_GEOM, i)
            if nm is None:
                nm = f"<unnamed_{i}>"
            names.append(nm)
        return names

    def print_all_geoms(self):
        """Print all geom names with ids (debug helper)."""
        names = self._list_geom_names()
        for i, nm in enumerate(names):
            print(f"[GMR][geom] id={i}: {nm}")

    @staticmethod
    def _geom_centers(data, geom1, geom2):
        return data.geom_xpos[geom1].copy(), data.geom_xpos[geom2].copy()

    def _geom_distance(self, model, data, geom1, geom2, distmax):
        fromto = np.zeros(6, dtype=float)
        try:
            dist = mj.mj_geomDistance(model, data, geom1, geom2, distmax, fromto)
        except TypeError:
            try:
                dist = mj.mj_geomDistance(model, data, geom1, geom2, fromto)
            except TypeError:
                dist = mj.mj_geomDistance(model, data, geom1, geom2)
        if isinstance(dist, (tuple, list)):
            if len(dist) >= 1:
                dist_val = dist[0]
            else:
                dist_val = dist
            if len(dist) >= 2:
                fromto = np.asarray(dist[1], dtype=float).reshape(-1)
            dist = dist_val
        if fromto.shape[0] >= 6:
            p1 = fromto[:3].copy()
            p2 = fromto[3:6].copy()
        else:
            p1, p2 = self._geom_centers(data, geom1, geom2)
        return float(dist), p1, p2

    def _point_jacobian(self, model, data, body_id, point):
        jacp = np.zeros((3, model.nv), dtype=float)
        jacr = np.zeros((3, model.nv), dtype=float)
        try:
            mj.mj_jac(model, data, jacp, jacr, point, body_id)
            return jacp
        except AttributeError:
            mj.mj_jacBody(model, data, jacp, jacr, body_id)
            body_pos = data.xpos[body_id]
            offset = point - body_pos
            # Point Jacobian = body Jacobian + angular Jacobian x offset.
            return jacp + np.cross(jacr.T, offset).T

    def compute_qp_inequalities(self, configuration, dt):
        model = configuration.model
        data = configuration.data
        nv = model.nv

        dist_min = self.minimum_distance_from_collisions
        dist_max = self.collision_detection_distance
        if dist_max <= dist_min:
            dist_max = dist_min + 1e-6

        G_rows = []
        h_rows = []

        for ids_a, ids_b, names_a, names_b in self.geom_pairs:
            best = None
            for geom1 in ids_a:
                for geom2 in ids_b:
                    dist, p1, p2 = self._geom_distance(model, data, geom1, geom2, dist_max)
                    if best is None or dist < best[0]:
                        best = (dist, p1, p2, geom1, geom2)

            if best is None:
                continue

            dist, p1, p2, geom1, geom2 = best
            if dist >= dist_max:
                continue

            direction = p2 - p1
            norm = np.linalg.norm(direction)
            if norm < 1e-9:
                p1, p2 = self._geom_centers(data, geom1, geom2)
                direction = p2 - p1
                norm = np.linalg.norm(direction)
                if norm < 1e-9:
                    continue
            direction /= norm

            body1 = model.geom_bodyid[geom1]
            body2 = model.geom_bodyid[geom2]
            J1 = self._point_jacobian(model, data, body1, p1)
            J2 = self._point_jacobian(model, data, body2, p2)
            Jd = direction @ (J2 - J1)

            phi = (dist - dist_min) / (dist_max - dist_min)
            h = self.gain * phi + self.bound_relaxation
            G_rows.append(-Jd)
            h_rows.append(h)

        if not G_rows:
            return Constraint(np.zeros((0, nv), dtype=float), np.zeros((0,), dtype=float))

        G = np.vstack(G_rows)
        h = np.asarray(h_rows, dtype=float)
        return Constraint(G, h)

class FootGroundContactLimit(Limit):
    """
    更精确的足部地面接触约束
    使用足部几何体的实际接触点
    """
    
    def __init__(self, model, foot_geom_names, min_height=0.0, margin=0.001, gain=1.0):
        self.model = model
        self.foot_geom_names = foot_geom_names if isinstance(foot_geom_names, list) else [foot_geom_names]
        self.min_height = min_height
        self.margin = margin
        self.gain = gain
        # 获取几何体ID
        self.geom_ids = []
        for name in self.foot_geom_names:
            geom_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
            if geom_id >= 0:
                self.geom_ids.append(geom_id)
                print(f"FootGroundContact: 注册几何体 {name} (ID: {geom_id})")
            else:
                print(f"警告: 未找到几何体 {name}")
    
    def compute_qp_inequalities(self, configuration, dt):
        model = configuration.model
        data = configuration.data
        nv = model.nv
        
        G_rows = []
        h_rows = []
        
        for idx, geom_id in enumerate(self.geom_ids):
                
            # 获取几何体位置（实际接触点）
            geom_pos = data.geom_xpos[geom_id].copy()
            height = min(model.geom_size[geom_id].copy())
            
            current_z = geom_pos[2]# - height
            
            if self.foot_geom_names[idx] == "pelvis_collision":
                current_z = geom_pos[2] - 0.01
            # if height < 1e-3:
            #     height = min(model.geom_size[geom_id].copy()[:2])
            # if height < 1e-3:
            #     height = model.geom_size[geom_id].copy()[0]
            if "knee" in self.foot_geom_names[idx]:
                current_z -= 0.075
            if "hand" in self.foot_geom_names[idx]:
                height = max(model.geom_size[geom_id].copy())
            # 如果已经在安全高度以上，跳过约束
            if current_z > self.min_height + self.margin:
                continue
            
            # 获取几何体所属的身体
            body_id = model.geom_bodyid[geom_id]
            
            # 获取接触点 Jacobian（更准确）
            jacp = np.zeros((3, nv))
            jacr = np.zeros((3, nv))
            mj.mj_jacGeom(model, data, jacp, jacr, geom_id)
            
            # 取 z 方向的 Jacobian
            Jz = jacp[2:3, :]  # 几何体位置在z方向的雅可比
            
            # 约束：z_new >= min_height + margin
            # z_new = current_z + Jz * dq >= min_height + margin
            # => Jz * dq >= (min_height + margin - current_z)
            # => -Jz * dq <= current_z - (min_height + margin)
            
            effective_min = self.min_height + self.margin
            G = -Jz
            h = self.gain*(current_z - effective_min)
            
            G_rows.append(G)
            h_rows.append(h)
            
        if not G_rows:
            return Constraint(np.zeros((0, nv)), np.zeros((0,)))
        
        G = np.vstack(G_rows)
        h = np.array(h_rows)
        return Constraint(G, h)

