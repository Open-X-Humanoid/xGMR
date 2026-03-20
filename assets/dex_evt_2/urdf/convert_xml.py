import mujoco

model = mujoco.MjModel.from_xml_path("/home/ps/develop/work/nov/dex_urdf/dex_evt/urdf/tiangong2dex_evt_mujoco.urdf")

mujoco.mj_saveLastXML("/home/ps/develop/work/nov/dex_urdf/dex_evt/urdf/tiangong2dex_evt_.xml",model)