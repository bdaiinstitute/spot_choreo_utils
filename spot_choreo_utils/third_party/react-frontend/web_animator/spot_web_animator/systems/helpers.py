# Copyright (c) 2023-2025 Boston Dynamics AI Institute LLC. All rights reserved.

from typing import List, Optional

from pydrake.common.value import AbstractValue, Value
from pydrake.geometry import (
    FramePoseVector,
    GeometryFrame,
    GeometryInstance,
    IllustrationProperties,
    Rgba,
    SceneGraph,
    SourceId,
    Sphere,
)
from pydrake.math import RigidTransform
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.framework import (
    Context,
    LeafSystem,
    ValueProducer,
)


class CenterOfMassVisualizer(LeafSystem):
    # The default radius and color for the center of mass visualization.
    _RADIUS = 0.03  # in meters.
    _COLOR = Rgba(1.0, 0, 0, 0.9)

    def __init__(
        self,
        plant: MultibodyPlant,
        scene_graph: SceneGraph,
        model_instances: Optional[List[ModelInstanceIndex]] = None,
    ):
        """Visualizes the center of mass of a plant for given model instances.

        Adds a red sphere as a visualization of the center of mass.
        Note, you will find the sphere under drake->illustrations. Once this is
        pushed to drake master, it will be easier to toggle it on and off from
        the menu.

        Args:
            plant: The multibody plant used to calculate the center of mass.
            scene_graph: The scene graph to which the visualization will be
                added to.
            model_instances: If provided, the center of mass for the given
                instances will be collectively computed.
        """
        super().__init__()
        self._plant = plant
        self._scene_graph = scene_graph
        self._model_instances = model_instances

        if self._model_instances is None:
            self._source_id = self._scene_graph.RegisterSource(f"{self._plant.get_name()}_com")
        else:
            model_instance_names = [self._plant.GetModelInstanceName(model_idx) for model_idx in self._model_instances]
            self._source_id = self._scene_graph.RegisterSource(f"{'_'.join(model_instance_names)}_com")

        self._com_frame = self._scene_graph.RegisterFrame(
            self._source_id,
            self._scene_graph.world_frame_id(),
            GeometryFrame("center_of_mass"),
        )

        geometry = GeometryInstance(RigidTransform(), Sphere(self._RADIUS), "com")
        props = IllustrationProperties()
        props.AddProperty("phong", "diffuse", self._COLOR)
        geometry.set_illustration_properties(props)

        self._scene_graph.RegisterGeometry(self._source_id, self._com_frame, geometry)

        self.plant_state_input_port = self.DeclareVectorInputPort(
            "plant_state",
            self._plant.num_multibody_states(),
        )

        self.DeclareAbstractOutputPort(
            "geometry_pose",
            lambda: AbstractValue.Make(FramePoseVector()),
            self.set_frame_pose_output,
        )

        # Declare cache entry for the multibody plant context.
        plant_context = self._plant.CreateDefaultContext()
        self._plant_context_cache = self.DeclareCacheEntry(
            "plant_context_cache",
            ValueProducer(
                allocate=Value(plant_context).Clone,
                calc=self.set_multibody_context,
            ),
            (
                {
                    self.input_port_ticket(self.plant_state_input_port.get_index()),
                }
            ),
        )

    def get_source_id(self) -> SourceId:
        """Returns the source id of the center of mass geometry."""
        return self._source_id

    def set_multibody_context(self, context: Context, abstract_value: Value) -> None:
        """Retrieves the plant state and sets the
        plant context cache.
        """
        plant_context = abstract_value.get_mutable_value()
        x_plant = self.plant_state_input_port.Eval(context)
        self._plant.SetPositionsAndVelocities(plant_context, x_plant)

    def set_frame_pose_output(self, context: Context, poses: FramePoseVector) -> None:
        """Computes the center of mass for the given model instances and sets
        the pose of the visualization geometry."""
        plant_context = self.get_cache_entry(self._plant_context_cache.cache_index()).Eval(context)

        if self._model_instances is None:
            p_world_com = self._plant.CalcCenterOfMassPositionInWorld(plant_context)
        else:
            p_world_com = self._plant.CalcCenterOfMassPositionInWorld(plant_context, self._model_instances)

        com_pose_vector = FramePoseVector()
        com_pose_vector.set_value(self._com_frame, RigidTransform(p=p_world_com))
        poses.set_value(com_pose_vector)
