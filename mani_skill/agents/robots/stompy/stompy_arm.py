from copy import deepcopy
from curses import echo
import numpy as np
import sapien
from transforms3d import euler

from mani_skill import PACKAGE_ASSET_DIR
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent
from mani_skill.sensors.camera import CameraConfig

#ROOT IS SOME x8 THING

# echo export CUDA_VISIBLE_DEVICES=0

@register_agent("stompy_arm")  # uncomment this if you want to register the agent so you can instantiate it by ID when creating environments
class StompyArm(BaseAgent):
    uid = "stompy_arm"
    urdf_path = f"{PACKAGE_ASSET_DIR}/robots/stompyarm/robot.urdf"

    urdf_config = dict(
        _materials=dict(
            gripper=dict(static_friction=2.0, dynamic_friction=2.0, restitution=0.0)
        ),
        link=dict(
            link_right_arm_1_hand_1_gripper_1=dict(
                material="gripper", patch_radius=0.1, min_patch_radius=0.1
            ),
            link_right_arm_1_hand_1_gripper_2=dict(
                material="gripper", patch_radius=0.1, min_patch_radius=0.1
            ),
        ),
    )
    startpos = [0.0, 0.0, 0.0]
    startorn = [0.0, 0.0, 0.0, 1.0]
    startrpy = [0.0, 0.0, 0.0]
    keyframes = dict(
        rest=Keyframe(
            pose = sapien.Pose(p=startpos, q=startorn), 
            qpos=np.array([1.0, 1.0, 1.0, 0.0, 0.0,100.0, 0.0, 0.0, 0.0]),
        )
    )
    # fix_root_link = True
    # balance_passive_force = True
    # load_multiple_collisions = True

    arm_joint_names = [
        'joint_x8_1_dof_x8', 
        'joint_x8_2_dof_x8', 
        'joint_x6_1_dof_x6', 
        'joint_x6_2_dof_x6', 
        'joint_x4_1_dof_x4', 
        'joint_hand_right_1_x4_3_dof_x4', 
        'joint_hand_right_1_x4_1_dof_x4', 
        'joint_hand_right_1_x4_2_dof_x4',
    ]

    gripper_joint_names = [
         'joint_hand_right_1_slider_3', 
         'joint_hand_right_1_slider_1',
    ]

    arm_joint_names = ["joint_x8_1_dof_x8", "joint_x8_2_dof_x8"]
    ee_link_name = "link_x6_1_outer_x6_1"

    arm_stiffness = 1e3
    arm_damping = 1e2
    arm_force_limit = 100

    gripper_stiffness = 1e3
    gripper_damping = 1e2
    gripper_force_limit = 100

    @property
    def _controller_configs(
        self,
    ):
        # print(self.robot.active_joints)
        # print(self.robot.active_joints_map.keys())
        # breakpoint()
        # print(self.ee_link_name)
        # breakpoint()
        arm_pd_ee_delta_pose = PDEEPoseControllerConfig(
            joint_names=self.arm_joint_names,
            pos_lower=-0.1,
            pos_upper=0.1,
            rot_lower=-0.1,
            rot_upper=0.1,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            ee_link=self.ee_link_name,
            urdf_path=self.urdf_path,
        )
        arm_pd_ee_target_delta_pose = deepcopy(arm_pd_ee_delta_pose)
        arm_pd_ee_target_delta_pose.use_target = True
        controller_configs = dict(
            pd_ee_delta_pose=dict(
                arm=arm_pd_ee_delta_pose
            ),
        )

        # Make a deepcopy in case users modify any config
        return deepcopy_dict(controller_configs)


    # ee_link_name = "link_hand_right_1_rack_1"

    # @property
    # def _controller_configs(
    #     self,
    # ):
    #     # print(self.robot.active_joints)
    #     # print(self.robot.active_joints_map.keys())
    #     # breakpoint()
    #     # print(self.ee_link_name)
    #     # breakpoint()
    #     arm_pd_ee_delta_pose = PDEEPoseControllerConfig(
    #         joint_names=self.arm_joint_names,
    #         pos_lower=-0.1,
    #         pos_upper=0.1,
    #         rot_lower=-0.1,
    #         rot_upper=0.1,
    #         stiffness=self.arm_stiffness,
    #         damping=self.arm_damping,
    #         force_limit=self.arm_force_limit,
    #         ee_link=self.ee_link_name,
    #         urdf_path=self.urdf_path,
    #     )
    #     arm_pd_ee_target_delta_pose = deepcopy(arm_pd_ee_delta_pose)
    #     arm_pd_ee_target_delta_pose.use_target = True
    #     breakpoint()
    #     gripper_pd_joint_pos = PDJointPosMimicControllerConfig(
    #         self.gripper_joint_names,
    #         lower=-0.01,  # a trick to have force when the object is thin
    #         upper=0.04,
    #         stiffness=self.gripper_stiffness,
    #         damping=self.gripper_damping,
    #         force_limit=self.gripper_force_limit,
    #     )
    #     controller_configs = dict(
    #         pd_ee_delta_pose=dict(
    #             arm=arm_pd_ee_delta_pose, gripper=gripper_pd_joint_pos
    #         ),
    #     )

    #     # Make a deepcopy in case users modify any config
    #     return deepcopy_dict(controller_configs)

    # def _after_init(self):
    #     # print(self.robot.get_links())
    #     pass

    # @property
    # def _controller_configs(
    #     self,
    # ):

    #     return dict(
    #         pd_joint_pos=dict(
    #             body=PDJointPosControllerConfig(
    #                 [x.name for x in self.robot.active_joints],
    #                 lower=None,
    #                 upper=None,
    #                 stiffness=100,
    #                 damping=10,
    #                 normalize_action=False,
    #             ),
    #             balance_passive_force=False,
    #         ),
    #         pd_joint_delta_pos=dict(
    #             body=PDJointPosControllerConfig(
    #                 [x.name for x in self.robot.active_joints],
    #                 lower=-0.1,
    #                 upper=0.1,
    #                 stiffness=20,
    #                 damping=5,
    #                 normalize_action=True,
    #                 use_delta=True,
    #             ),
    #             balance_passive_force=False,
    #         ),
    #     )


    # @property
    # def _sensor_configs(self):
    #     return [
    #         CameraConfig(
    #             uid="head_camera",
    #             pose=sapien.Pose(
    #                 p=[0.12, 0, 0.02], q=euler.euler2quat(-np.pi / 2, 0, 0)
    #             ),
    #             width=128,
    #             height=128,
    #             fov=1.57,
    #             near=0.01,
    #             far=100,
    #             entity_uid="link_head_1_head_1",  # mount cameras relative to existing link IDs as so
    #         )
    #     ]
