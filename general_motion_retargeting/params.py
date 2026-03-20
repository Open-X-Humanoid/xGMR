import pathlib

HERE = pathlib.Path(__file__).parent
IK_CONFIG_ROOT = HERE / "ik_configs"
ASSET_ROOT = HERE / ".." / "assets"

ROBOT_XML_DICT = {
    "unitree_g1": ASSET_ROOT / "unitree_g1" / "g1_mocap_29dof.xml",
    "unitree_g1_with_hands": ASSET_ROOT / "unitree_g1" / "g1_mocap_29dof_with_hands.xml",
    "unitree_h1": ASSET_ROOT / "unitree_h1" / "h1.xml",
    "unitree_h1_2": ASSET_ROOT / "unitree_h1_2" / "h1_2_handless.xml",
    "booster_t1": ASSET_ROOT / "booster_t1" / "T1_serial.xml",
    "booster_t1_29dof": ASSET_ROOT / "booster_t1_29dof" / "t1_mocap.xml",
    "stanford_toddy": ASSET_ROOT / "stanford_toddy" / "toddy_mocap.xml",
    "fourier_n1": ASSET_ROOT / "fourier_n1" / "n1_mocap.xml",
    "engineai_pm01": ASSET_ROOT / "engineai_pm01" / "pm_v2.xml",
    "kuavo_s45": ASSET_ROOT / "kuavo_s45" / "biped_s45_collision.xml",
    "hightorque_hi": ASSET_ROOT / "hightorque_hi" / "hi_25dof.xml",
    "galaxea_r1pro": ASSET_ROOT / "galaxea_r1pro" / "r1_pro.xml",
    "berkeley_humanoid_lite": ASSET_ROOT / "berkeley_humanoid_lite" / "bhl_scene.xml",
    "booster_k1": ASSET_ROOT / "booster_k1" / "K1_serial.xml",
    "pnd_adam_lite": ASSET_ROOT / "pnd_adam_lite" / "scene.xml",
    "tienkung2": ASSET_ROOT / "tienkung2_pro" / "urdf" / "tiangong2_EVT_V2.xml",
    "tienkung2_lite": ASSET_ROOT / "tienkung2_lite" / "urdf" / "tiangong2_Lite.xml",
    "tienkung2_plus": ASSET_ROOT / "tienkung2_plus" / "urdf" / "tiangong2_EVT_V9_urdf.xml",
    "dex0902": ASSET_ROOT / "dex0902" / "urdf" /"dex0902.xml",
    "dex_evt2": ASSET_ROOT / "dex_evt_2" / "urdf" /"dex_evt.xml",
}

ROBOT_URDF_DICT = {
    "dex_evt2": ASSET_ROOT / "dex_evt_2" / "urdf" /"tiangong2dex_no_contact.urdf",
    "tienkung2": ASSET_ROOT / "tienkung2_pro" / "urdf" / "tiangong2_EVT_V2.urdf",
    "tienkung2_plus": ASSET_ROOT / "tienkung2_plus" / "urdf" / "tiangong2_EVT_V9_urdf.urdf",
    "tienkung2_lite": ASSET_ROOT / "tienkung2_lite" / "urdf" / "tiangong2_Lite.urdf",
}

IK_CONFIG_DICT = {
    # offline data
    "smplx":{
        "unitree_g1": IK_CONFIG_ROOT / "smplx_to_g1.json",
        "unitree_g1_with_hands": IK_CONFIG_ROOT / "smplx_to_g1.json",
        "unitree_h1": IK_CONFIG_ROOT / "smplx_to_h1.json",
        "unitree_h1_2": IK_CONFIG_ROOT / "smplx_to_h1_2.json",
        "booster_t1": IK_CONFIG_ROOT / "smplx_to_t1.json",
        "booster_t1_29dof": IK_CONFIG_ROOT / "smplx_to_t1_29dof.json",
        "stanford_toddy": IK_CONFIG_ROOT / "smplx_to_toddy.json",
        "fourier_n1": IK_CONFIG_ROOT / "smplx_to_n1.json",
        "engineai_pm01": IK_CONFIG_ROOT / "smplx_to_pm01.json",
        "kuavo_s45": IK_CONFIG_ROOT / "smplx_to_kuavo.json",
        "hightorque_hi": IK_CONFIG_ROOT / "smplx_to_hi.json",
        "galaxea_r1pro": IK_CONFIG_ROOT / "smplx_to_r1pro.json",
        "berkeley_humanoid_lite": IK_CONFIG_ROOT / "smplx_to_bhl.json",
        "booster_k1": IK_CONFIG_ROOT / "smplx_to_k1.json",
        "pnd_adam_lite": IK_CONFIG_ROOT / "smplx_to_adam.json",
        "tienkung": IK_CONFIG_ROOT / "smplx_to_tienkung.json",
    },
    "bvh":{
        "unitree_g1": IK_CONFIG_ROOT / "bvh_to_g1.json",
        "unitree_g1_with_hands": IK_CONFIG_ROOT / "bvh_to_g1.json",
        "booster_t1_29dof": IK_CONFIG_ROOT / "bvh_to_t1_29dof.json",
        "fourier_n1": IK_CONFIG_ROOT / "bvh_to_n1.json",
        "stanford_toddy": IK_CONFIG_ROOT / "bvh_to_toddy.json",
        "engineai_pm01": IK_CONFIG_ROOT / "bvh_to_pm01.json",
        "dex0902": IK_CONFIG_ROOT / "bvh_to_dex_v1.json",
        "dex_evt_noitom": IK_CONFIG_ROOT / "bvh_noitom_to_dex_evt.json",
        "dex_evt_xsens": IK_CONFIG_ROOT / "bvh_xsens_to_dex_evt.json",
        "dex_evt_lafan": IK_CONFIG_ROOT / "bvh_lafan_to_dex_evt.json",
        
        "tienkung2_noitom": IK_CONFIG_ROOT / "bvh_noitom_to_tienkung.json",
        "tienkung2_xsens": IK_CONFIG_ROOT / "bvh_xsens_to_tienkung.json",
        "tienkung2_lafan": IK_CONFIG_ROOT / "bvh_lafan_to_tienkung.json",

        "tienkung2_lite_noitom": IK_CONFIG_ROOT / "bvh_noitom_to_tienkung_lite.json",
        "tienkung2_lite_xsens": IK_CONFIG_ROOT / "bvh_xsens_to_tienkung_lite.json",

        "tienkung2_plus_noitom": IK_CONFIG_ROOT / "bvh_noitom_to_tienkung_plus.json",
        "tienkung2_plus_xsens": IK_CONFIG_ROOT / "bvh_xsens_to_tienkung_plus.json",
        "tienkung2_plus_lafan": IK_CONFIG_ROOT / "bvh_lafan_to_tienkung_plus.json",

    },
    "fbx":{
        "unitree_g1": IK_CONFIG_ROOT / "fbx_to_g1.json",
        "unitree_g1_with_hands": IK_CONFIG_ROOT / "fbx_to_g1.json",
    },
    "fbx_offline":{
        "unitree_g1": IK_CONFIG_ROOT / "fbx_offline_to_g1.json",
    },
}


ROBOT_BASE_DICT = {
    "unitree_g1": "pelvis",
    "unitree_g1_with_hands": "pelvis",
    "unitree_h1": "pelvis",
    "unitree_h1_2": "pelvis",
    "booster_t1": "Waist",
    "booster_t1_29dof": "Waist",
    "stanford_toddy": "waist_link",
    "fourier_n1": "base_link",
    "engineai_pm01": "LINK_BASE",
    "kuavo_s45": "base_link",
    "hightorque_hi": "base_link",
    "galaxea_r1pro": "torso_link4",
    "berkeley_humanoid_lite": "imu_2",
    "booster_k1": "Trunk",
    "pnd_adam_lite": "pelvis",
    "tienkung2": "pelvis",
    "dex0902": "base_link",
    "dex_v3": "pelvis",
    "dex_evt": "pelvis",
}

VIEWER_CAM_DISTANCE_DICT = {
    "unitree_g1": 2.0,
    "unitree_g1_with_hands": 2.0,
    "unitree_h1": 3.0,
    "unitree_h1_2": 3.0,
    "booster_t1": 2.0,
    "booster_t1_29dof": 2.0,
    "stanford_toddy": 1.0,
    "fourier_n1": 2.0,
    "engineai_pm01": 2.0,
    "kuavo_s45": 3.0,
    "hightorque_hi": 2.0,
    "galaxea_r1pro": 3.0,
    "berkeley_humanoid_lite": 2.0,
    "booster_k1": 2.0,
    "pnd_adam_lite": 3.0,
    "tienkung2": 3.0,
    "dex0902": 3.0,
    "dex_v3" : 3.0,
    "dex_evt": 3.0,
}
