# Copyright (c) 2023-2025 Boston Dynamics AI Institute LLC. All rights reserved.

from enum import Enum
from pathlib import Path
from typing import Dict, Optional, Union

import numpy as np
import yaml
from pydrake.all import MultibodyPlantConfig
from pydrake.common.containers import namedview
from pydrake.geometry import (
    CollisionFilterDeclaration,
    GeometrySet,
    Meshcat,
    Role,
    SceneGraph,
)
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import ApplyMultibodyPlantConfig, MultibodyPlant
from pydrake.multibody.tree import PdControllerGains
from pydrake.systems.framework import Context, Diagram, DiagramBuilder
from pydrake.systems.primitives import (
    PassThrough,
)
from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig
from spot_web_animator.systems.helpers import CenterOfMassVisualizer


def get_curr_path() -> str:
    return str(Path(__file__).parent)


class BaseActuatorName(Enum):
    FL_HX = "front_left_hip_x_motor"
    FL_HY = "front_left_hip_y_motor"
    FL_KN = "front_left_knee_motor"
    FR_HX = "front_right_hip_x_motor"
    FR_HY = "front_right_hip_y_motor"
    FR_KN = "front_right_knee_motor"
    HL_HX = "rear_left_hip_x_motor"
    HL_HY = "rear_left_hip_y_motor"
    HL_KN = "rear_left_knee_motor"
    HR_HX = "rear_right_hip_x_motor"
    HR_HY = "rear_right_hip_y_motor"
    HR_KN = "rear_right_knee_motor"


class ArmActuatorName(Enum):
    ARM_sh0 = "arm_sh0_motor"
    ARM_sh1 = "arm_sh1_motor"
    ARM_el0 = "arm_el0_motor"
    ARM_el1 = "arm_el1_motor"
    ARM_wr0 = "arm_wr0_motor"
    ARM_wr1 = "arm_wr1_motor"


class GripperActuatorName(Enum):
    ARM_f1x = "arm_f1x_motor"


SpotPDGainsType = Dict[
    Union[BaseActuatorName, ArmActuatorName, GripperActuatorName],
    PdControllerGains,
]


class FootCenterFrame(Enum):
    FRONT_LEFT = "front_left_foot_center"
    FRONT_RIGHT = "front_right_foot_center"
    REAR_LEFT = "rear_left_foot_center"
    REAR_RIGHT = "rear_right_foot_center"


def read_default_joint_configurations() -> Dict:
    """Returns the loaded joint configurations."""
    with open(get_curr_path() + "/config/joint_configurations.yaml", "r") as f:
        joint_configuration = yaml.safe_load(f)
    return joint_configuration


def stub_joint_gains() -> SpotPDGainsType:
    """Gains aren't used for Web Animator. Introduce Stubs."""
    all_actuators = list(BaseActuatorName) + list(ArmActuatorName) + list(GripperActuatorName)
    return {
        acutator_name: PdControllerGains(
            p=1,
            d=1,
        )
        for acutator_name in all_actuators
    }


class SpotModel:
    def __init__(self, sim_time_step: float = 0.001, has_arm: bool = False) -> None:
        """Provides access to the Spot plant and helper functions.

        Args:
            sim_time_step: For a time step (seconds) greater than zero, the
                system will be discrete with periodic updates. To choose
                a continuous system, the time step should equal exactly zero.
            has_arm: If true, it will weld the spot robot arm to its body.
        """
        self.has_arm = has_arm
        self._plant = MultibodyPlant(time_step=sim_time_step)
        plant_config = MultibodyPlantConfig(time_step=sim_time_step, discrete_contact_approximation="similar")
        ApplyMultibodyPlantConfig(plant_config, self._plant)

        self._scene_graph = SceneGraph()
        self._plant.RegisterAsSourceForSceneGraph(self._scene_graph)

        # Loading models.
        parser = Parser(self._plant)
        parser.package_map().AddPackageXml(get_curr_path() + "/external/spot_description/package.xml")
        self.has_arm = has_arm
        self.base_instance = parser.AddModels(get_curr_path() + "/external/spot_description/urdf/out/spot.urdf")[0]
        self.arm_instance = None
        self.gripper_instance = None
        if has_arm:
            self.arm_instance = parser.AddModels(
                get_curr_path() + "/external/spot_description/urdf/out/standalone_arm.urdf"
            )[0]
            self._plant.WeldFrames(
                self._plant.GetFrameByName("body", self.base_instance),
                self._plant.GetFrameByName("body", self.arm_instance),
            )

            self.gripper_instance = parser.AddModels(
                get_curr_path() + "/external/spot_description/urdf/out/standalone_gripper.urdf"
            )[0]
            self._plant.WeldFrames(
                self._plant.GetFrameByName("arm_link_wr1", self.arm_instance),
                self._plant.GetFrameByName("arm_link_wr1", self.gripper_instance),
            )

            # Filter collisions between arm shoulder link and the body link,
            # since we welded those geometries together.
            # Similarly, we will filter the collisions between gripper
            # and the wrist.

            filter_manager = self._scene_graph.collision_filter_manager()
            inspector = self._scene_graph.model_inspector()

            body_and_shoulder_geometry_ids_to_filter = []
            wrist_and_finger_geometry_ids_to_filter = []
            wrist_and_jaw_geometry_ids_to_filter = []
            for gid in inspector.GetGeometryIds(GeometrySet(inspector.GetAllGeometryIds()), Role.kProximity):
                gid_name = inspector.GetName(inspector.GetFrameId(gid))
                if "body" in gid_name or "arm_link_sh0" in gid_name:
                    body_and_shoulder_geometry_ids_to_filter.append(gid)

                if "arm_link_wr1" in gid_name or "arm_link_fngr" in gid_name:
                    wrist_and_finger_geometry_ids_to_filter.append(gid)

                if "arm_link_wr1" in gid_name or "arm_link_jaw" in gid_name:
                    wrist_and_jaw_geometry_ids_to_filter.append(gid)

            filter_manager.Apply(
                CollisionFilterDeclaration().ExcludeWithin(GeometrySet(body_and_shoulder_geometry_ids_to_filter))
            )

            filter_manager.Apply(
                CollisionFilterDeclaration().ExcludeWithin(GeometrySet(wrist_and_finger_geometry_ids_to_filter))
            )

            filter_manager.Apply(
                CollisionFilterDeclaration().ExcludeWithin(GeometrySet(wrist_and_jaw_geometry_ids_to_filter))
            )

        # Set the default joint pd gains.
        default_gains = stub_joint_gains()
        self.set_joint_pd_gains(default_gains)

        # Named view's.
        self.BaseStateView: namedview = None
        self.ArmStateView: namedview = None
        self.GripperStateView: namedview = None

    def get_mutable_multibody_plant(self) -> MultibodyPlant:
        """Returns a mutable plant of the system.

        Returns:
            The main plant responsible for the dynamics of the robot and the
            environment. This can be used to, e.g., add additional elements
            into the world before calling Finalize().
        """
        return self._plant

    def get_mutable_scene_graph(self) -> SceneGraph:
        """Returns a mutable SceneGraph of the system.

        Returns:
            The SceneGraph responsible for all of the geometry for the robot
            and the environment. This can be used to, e.g., add additional
            elements into the world before calling Finalize().
        """
        return self._scene_graph

    def num_base_positions(self) -> int:
        """Returns the size of position vector q for the base."""
        return self._plant.num_positions(self.base_instance)

    def num_base_velocities(self) -> int:
        """Returns the size of velocity vector v for the base."""
        return self._plant.num_velocities(self.base_instance)

    def num_base_joints(self) -> int:
        """Returns the number of joints in the base."""
        return self._plant.num_actuators(self.base_instance)

    def num_arm_positions(self) -> int:
        """Returns the size of position vector q for the arm."""
        if self.has_arm:
            return self._plant.num_positions(self.arm_instance)
        else:
            raise ValueError("Spot instance does not have an arm.")

    def num_arm_velocities(self) -> int:
        """Returns the size of velocity vector v for the arm."""
        if self.has_arm:
            return self._plant.num_velocities(self.arm_instance)
        else:
            raise ValueError("Spot instance does not have an arm.")

    def num_arm_joints(self) -> int:
        """Returns the number of joints in the arm."""
        if self.has_arm:
            return self._plant.num_actuators(self.arm_instance)
        else:
            raise ValueError("Spot instance does not have an arm.")

    def num_gripper_positions(self) -> int:
        """Returns the size of position vector q for the gripper."""
        if self.has_arm:
            return self._plant.num_positions(self.gripper_instance)
        else:
            raise ValueError("Spot instance does not have an arm.")

    def num_gripper_velocities(self) -> int:
        """Returns the size of velocity vector v for the gripper."""
        if self.has_arm:
            return self._plant.num_velocities(self.gripper_instance)
        else:
            raise ValueError("Spot instance does not have an arm.")

    def num_gripper_joints(self) -> int:
        """Returns the number of joints in the gripper."""
        if self.has_arm:
            return self._plant.num_actuators(self.gripper_instance)
        else:
            raise ValueError("Spot instance does not have an arm.")

    def set_default_base_configuration(self) -> None:
        """Sets the plants default position to the desired base configuration.

        This sets the default free body pose and the joint angles of the
        base accordingly.

        Args:
            configuration: The desired base configuration.
        """
        # Read configuration file.
        base_configuration = read_default_joint_configurations()["base"]
        translation = base_configuration["stand"]["translation"]
        joint_positions = base_configuration["stand"]["joint_positions"]

        # Set floating body pose.
        self._plant.GetJointByName("body", self.base_instance).set_default_positions([1, 0, 0, 0] + translation)

        # Set joint angles.
        for joint_name, joint_angle in joint_positions.items():
            self._plant.GetJointByName(joint_name, self.base_instance).set_default_angle(joint_angle)

    def set_default_arm_configuration(self) -> None:
        """Sets the plants default position to the desired arm configuration.

        This sets the default joint angles of the arm accordingly.

        Args:
            configuration: The desired arm configuration.
        """
        if not self.has_arm:
            raise ValueError("Spot instance does not have an arm.")

        # Read configuration file.
        arm_configuration = read_default_joint_configurations()["arm"]
        joint_positions = arm_configuration["stow"]["joint_positions"]

        # Set joint angles.
        for joint_name, joint_angle in joint_positions.items():
            self._plant.GetJointByName(joint_name, self.arm_instance).set_default_angle(joint_angle)

    def set_default_gripper_configuration(self) -> None:
        """Sets the plants default position to the desired gripper configuration.

        This sets the default joint angles of the gripper accordingly.

        Args:
            configuration: The desired gripper configuration.
        """
        if not self.has_arm:
            raise ValueError("Spot instance does not have an arm.")

        # Read configuration file.
        gripper_configuration = read_default_joint_configurations()["gripper"]
        joint_positions = gripper_configuration["closed"]["joint_positions"]

        # Set joint angles.
        for joint_name, joint_angle in joint_positions.items():
            self._plant.GetJointByName(joint_name, self.gripper_instance).set_default_angle(joint_angle)

    def get_default_base_state(self) -> namedview:
        """Extracts the default base state.

        Returns:
            An instantiation of the base state view
            with the extracted state.
        """
        q_base = self._plant.GetDefaultPositions(self.base_instance)
        qDt_base = np.zeros(self.num_base_velocities())
        x_base = np.hstack([q_base, qDt_base])
        named_base_state = self.BaseStateView(x_base)
        return named_base_state

    def get_default_arm_state(self) -> namedview:
        """Extracts the default arm state.

        Returns:
            An instantiation of the arm state view
            with the extracted state.
        """
        q_arm = self._plant.GetDefaultPositions(self.arm_instance)
        qDt_arm = np.zeros(self.num_arm_velocities())
        x_arm = np.hstack([q_arm, qDt_arm])
        named_arm_state = self.ArmStateView(x_arm)
        return named_arm_state

    def get_default_gripper_state(self) -> namedview:
        """Extracts the default gripper state.

        Returns:
            An instantiation of the gripper state view
            with the extracted state.
        """
        q_gripper = self._plant.GetDefaultPositions(self.gripper_instance)
        qDt_gripper = np.zeros(self.num_gripper_velocities())
        x_gripper = np.hstack([q_gripper, qDt_gripper])
        named_gripper_state = self.GripperStateView(x_gripper)
        return named_gripper_state

    def get_base_state(self, spot_context: Context) -> namedview:
        """Extracts the base state from the context.
        Args:
            spot_context: A context with the spot robot.

        Returns:
            An instantiation of the base state view
            with the extracted state.
        """
        base_state = self._plant.GetPositionsAndVelocities(spot_context, self.base_instance)
        named_base_state = self.BaseStateView(base_state)
        return named_base_state

    def get_arm_state(self, spot_context: Context) -> namedview:
        """Extracts the arm state from the context.
        Args:
            spot_context: A context with the spot robot.

        Returns:
            An instantiation of the arm state view
            with the extracted state.
        """
        arm_state = self._plant.GetPositionsAndVelocities(spot_context, self.arm_instance)
        named_arm_state = self.ArmStateView(arm_state)
        return named_arm_state

    def get_gripper_state(self, spot_context: Context) -> namedview:
        """Extracts the gripper state from the context.
        Args:
            spot_context: A context with the spot robot.

        Returns:
            An instantiation of the gripper state view
            with the extracted state.
        """
        gripper_state = self._plant.GetPositionsAndVelocities(spot_context, self.gripper_instance)
        named_gripper_state = self.GripperStateView(gripper_state)
        return named_gripper_state

    def set_joint_pd_gains(self, joint_pd_gains: SpotPDGainsType) -> None:
        """Sets the plants PD gains for the internal position controller.

        The internal PD controller of the multibody plant is implemented as:
        u = Kp⋅(qd - q) + Kd⋅(vd - v) + u_ff
        u = max(-e, min(e, ũ))

        Args:
            joint_pd_gains: A dictionary mapping from actuator name to the PD
            controller gains. Gains for the arm and gripper will be silently
            ignored if Spot has no arm.

        Raises:
            Exception: Plant was already finalized.

        """
        if self._plant.is_finalized():
            raise Exception("Plant was already finalized.")

        for actuator_name, pd_gain in joint_pd_gains.items():
            if not self.has_arm and actuator_name not in BaseActuatorName:
                continue

            actuator = self._plant.GetJointActuatorByName(actuator_name.value)
            actuator.set_controller_gains(pd_gain)

    def SetFreeBodyPose(self, spot_context: Context, X_WB: RigidTransform) -> None:
        """Sets the context to store the pose of the spot base.

        Args:
            spot_context: A context with the spot robot.
            X_WB: The transformation from world to the 'body' frame of spot.
        """
        self._plant.SetFreeBodyPose(
            spot_context,
            self._plant.GetBodyByName("body", self.base_instance),
            X_WB,
        )

    def Finalize(
        self,
    ) -> None:
        """This finalizes the model plant and sets model attributes."""
        self._plant.Finalize()

        # Only after finalizing the plant, the state views can be populated.
        self.BaseStateView = namedview(
            "base_state",
            self._plant.GetPositionNames(
                add_model_instance_prefix=False,
                model_instance=self.base_instance,
            )
            + self._plant.GetVelocityNames(
                add_model_instance_prefix=False,
                model_instance=self.base_instance,
            ),
        )

        if self.has_arm:
            self.ArmStateView = namedview(
                "arm_state",
                self._plant.GetPositionNames(
                    add_model_instance_prefix=False,
                    model_instance=self.arm_instance,
                )
                + self._plant.GetVelocityNames(
                    add_model_instance_prefix=False,
                    model_instance=self.arm_instance,
                ),
            )

            self.GripperStateView = namedview(
                "gripper_state",
                self._plant.GetPositionNames(
                    add_model_instance_prefix=False,
                    model_instance=self.gripper_instance,
                )
                + self._plant.GetVelocityNames(
                    add_model_instance_prefix=False,
                    model_instance=self.gripper_instance,
                ),
            )


class MockSpot(Diagram):
    def __init__(
        self,
        sim_time_step: float = 0.001,
        has_arm: bool = False,
    ):
        """Initiates as diagram with the Spot robot.

        Args:
            sim_time_step: For a time step (seconds) greater than zero, the
                system will be discrete with periodic updates. To choose
                a continuous system, the time step should equal exactly zero.
            has_arm: If true, it will weld the spot robot arm to its body.
        """
        super().__init__()
        self.model = SpotModel(sim_time_step, has_arm)
        self._scene_graph = self.model.get_mutable_scene_graph()
        self._plant = self.model.get_mutable_multibody_plant()
        self.set_name("MockSpot")

    def Finalize(
        self,
        meshcat: Optional[Meshcat] = None,
        builder: Optional[DiagramBuilder] = None,
        output_delay: int = 0,
    ) -> None:
        """This finalizes the plant and builds the diagram.

        Users *must* call Finalize() after making any additions to the
        multibody plant and before using this class in the Systems framework.
        This should be called exactly once.

        Args:
            meshcat: Optionally pass in a meshcat instance to add default
                visualization configurations.
                This calls ApplyVisalizationConfig().
                Note, due to a bug in drake, all sliders in the meshcat
                instance will be deleted.
            builder: One can provide a builder with previously added systems.
            output_delay: Defines the discrete delay of all output ports in
                milliseconds. The loggers record after the delay system.
        """
        # Finalize the plant.
        self.model.Finalize()

        # Set default configurations.
        self.model.set_default_base_configuration()
        if self.model.has_arm:
            self.model.set_default_arm_configuration()
            self.model.set_default_gripper_configuration()

        if builder is None:
            builder = DiagramBuilder()

        builder.AddNamedSystem("SpotPlant", self._plant)
        builder.AddNamedSystem("SpotSceneGraph", self._scene_graph)

        builder.Connect(
            self._plant.get_geometry_poses_output_port(),
            self._scene_graph.get_source_pose_port(self._plant.get_source_id()),
        )
        builder.Connect(
            self._scene_graph.get_query_output_port(),
            self._plant.get_geometry_query_input_port(),
        )

        # Export inputs for each subsystem.
        # Desired base state input.
        default_base_joint_state = np.zeros(2 * self.model.num_base_joints())
        default_base_joint_state[: self.model.num_base_joints()] = self.model.get_default_base_state()[
            7 : self.model.num_base_positions()
        ]
        default_base_joint_state[self.model.num_base_joints() :] = self.model.get_default_base_state()[
            self.model.num_base_positions() + 6 :
        ]
        desired_base_state_input = builder.AddSystem(PassThrough(default_base_joint_state))

        builder.Connect(
            desired_base_state_input.get_output_port(),
            self._plant.get_desired_state_input_port(self.model.base_instance),
        )
        builder.ExportInput(
            desired_base_state_input.get_input_port(0),
            "desired_base_state",
        )

        if self.model.has_arm:
            # Export the arm and gripper input separately.
            # Desired arm state input.
            desired_arm_state_input = builder.AddSystem(PassThrough(self.model.get_default_arm_state()[:]))

            builder.Connect(
                desired_arm_state_input.get_output_port(),
                self._plant.get_desired_state_input_port(self.model.arm_instance),
            )

            builder.ExportInput(
                desired_arm_state_input.get_input_port(0),
                "desired_arm_state",
            )

            # Desired gripper state input.
            desired_gripper_state_input = builder.AddSystem(PassThrough(self.model.get_default_gripper_state()[:]))

            builder.Connect(
                desired_gripper_state_input.get_output_port(),
                self._plant.get_desired_state_input_port(self.model.gripper_instance),
            )

            builder.ExportInput(
                desired_gripper_state_input.get_input_port(0),
                "desired_gripper_state",
            )

        # Export the base state.
        base_state_output = builder.AddSystem(
            PassThrough(
                self.model.get_default_base_state()[:],
            )
        )

        builder.Connect(
            self._plant.get_state_output_port(self.model.base_instance),
            base_state_output.get_input_port(),
        )
        builder.ExportOutput(
            base_state_output.get_output_port(),
            "base_state",
        )

        if self.model.has_arm:
            arm_state_output = builder.AddSystem(
                PassThrough(
                    self.model.get_default_arm_state()[:],
                )
            )

            builder.Connect(
                self._plant.get_state_output_port(self.model.arm_instance),
                arm_state_output.get_input_port(),
            )
            builder.ExportOutput(
                arm_state_output.get_output_port(),
                "arm_state",
            )

            gripper_state_output = builder.AddSystem(
                PassThrough(
                    self.model.get_default_gripper_state()[:],
                )
            )

            builder.Connect(
                self._plant.get_state_output_port(self.model.gripper_instance),
                gripper_state_output.get_input_port(),
            )
            builder.ExportOutput(
                gripper_state_output.get_output_port(),
                "gripper_state",
            )

        builder.ExportOutput(self._scene_graph.get_query_output_port(), "geometry_query")

        if meshcat:
            # Add default visualization configurations to meshcat.
            ApplyVisualizationConfig(
                config=VisualizationConfig(
                    publish_inertia=True,
                    publish_contacts=True,
                    enable_alpha_sliders=False,
                ),
                plant=self._plant,
                scene_graph=self._scene_graph,
                builder=builder,
                meshcat=meshcat,
            )

            # Add center of mass visualization.
            com_visualizer = builder.AddSystem(
                CenterOfMassVisualizer(
                    self._plant,
                    self._scene_graph,
                    (
                        [
                            self.model.base_instance,
                            self.model.arm_instance,
                            self.model.gripper_instance,
                        ]
                        if self.model.has_arm
                        else [self.model.base_instance]
                    ),
                )
            )
            builder.Connect(
                self._plant.get_state_output_port(),
                com_visualizer.get_input_port(),
            )
            builder.Connect(
                com_visualizer.get_output_port(),
                self._scene_graph.get_source_pose_port(com_visualizer.get_source_id()),
            )

        builder.BuildInto(self)
