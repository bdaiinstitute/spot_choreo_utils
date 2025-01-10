#  Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.
import logging
import unittest

from bosdyn.api.spot.choreography_params_pb2 import AnimateParams
from bosdyn.api.spot.choreography_sequence_pb2 import (
    ChoreographySequence,
    MoveParams,
)
from test_animation_builder import GRIPPER_ANIMATION_NAME, simple_gripper_animation

from spot_choreo_utils.choreo_creation.choreo_builders.animation_builder import AnimationBuilder

# Functions tested in this file
from spot_choreo_utils.choreo_creation.choreo_builders.animation_operators import (
    adjust_keyframes_to_timestamps,
    create_single_animation_sequence,
    ensure_animation_ends_standing_stable,
    find_last_standing_keyframe,
)


def simple_gripper_sequence() -> ChoreographySequence:
    SEQUENCE_NAME = GRIPPER_ANIMATION_NAME
    ANIMATION_NAME = GRIPPER_ANIMATION_NAME
    # This paramater is more important within the Choreographer GUI so you can calculate how
    # BPMs translate into slices and set one keyframe for each 1/4 of a beat. For procedural animations
    # we set the time directly so you might as well have an arbitrarily high precision for slices_per_minute
    SLICES_PER_MINUTE = 120
    # Requested slices doesn't matter for a sequence that has only one move
    # For multi-move sequences this should be claculated with slices per minute and animation length
    REQUESTED_SLICES = 4

    animation_params = AnimateParams()
    animation_params.animation_name = ANIMATION_NAME

    animation_move = MoveParams()
    animation_move.type = "animation"
    # Must be > 0
    animation_move.requested_slices = REQUESTED_SLICES
    animation_move.animate_params.CopyFrom(animation_params)

    simple_seqeuence = ChoreographySequence()
    simple_seqeuence.name = SEQUENCE_NAME
    simple_seqeuence.slices_per_minute = SLICES_PER_MINUTE
    simple_seqeuence.moves.extend([animation_move])

    return simple_seqeuence


def create_copy_of_timestamps(animation_builder: AnimationBuilder) -> list[float]:
    return [keyframe.time for keyframe in animation_builder._animation.animation_keyframes]


class AnimationBuilderUtilsTester(unittest.TestCase):
    def setUp(self) -> None:
        self._logger = logging.Logger("AnimationBuilderTestLogger")

    def test_animation_to_sequence_transformations(self) -> None:
        """
        Checks functions that take animations and transform them into builders and sequences quickly
        """
        build_settings = AnimationBuilder.BuildSettings()
        build_settings.apply_unique_name = False
        build_settings.hold_final_pose_s = 0

        gripper_anim_original = simple_gripper_animation()
        gripper_anim_builder = AnimationBuilder.from_animation(gripper_anim_original)
        self.assertEqual(gripper_anim_original, gripper_anim_builder.build(build_settings))

        original_sequence = simple_gripper_sequence()
        (new_animation, new_sequence) = create_single_animation_sequence(gripper_anim_builder, build_settings)
        self.assertEqual(new_animation, gripper_anim_original)
        self.assertEqual(original_sequence, new_sequence)

    def test_robot_standing_helpers(self) -> None:
        """
        Test functions that make sure the robot is standing at the end of each animation
        """
        simple_gripper_builder = AnimationBuilder.from_animation(simple_gripper_animation())
        self.assertEqual(
            find_last_standing_keyframe(simple_gripper_builder),
            len(simple_gripper_builder._animation.animation_keyframes) - 1,
        )

        build_settings = AnimationBuilder.BuildSettings()
        build_settings.apply_unique_name = False
        build_settings.hold_final_pose_s = 0

        original_animation = simple_gripper_builder.build(build_settings)
        ensure_animation_ends_standing_stable(simple_gripper_builder)

        # Check that the animation was unaltered since it's already standing fine
        self.assertEqual(original_animation, simple_gripper_builder.build(build_settings))

    def test_animation_append(self) -> None:
        """
        Test addding two animations together to create a new one
        """
        simple_gripper_builder_one = AnimationBuilder.from_animation(simple_gripper_animation())
        simple_gripper_builder_two = AnimationBuilder.from_animation(simple_gripper_animation())

        delay = 3

        simple_gripper_builder_one.append_animation(simple_gripper_builder_two, delay=delay)

        build_settings = AnimationBuilder.BuildSettings()
        build_settings.hold_final_pose_s = 0

        # Check that the append worked
        combined_anim = simple_gripper_builder_one.build().animation_keyframes
        anim_one = simple_gripper_builder_one.build(build_settings)
        anim_two = simple_gripper_builder_two.build(build_settings)
        final_time_one = anim_one.animation_keyframes[-1].time

        for key_idx, keyframe in enumerate(combined_anim):
            if keyframe.time <= final_time_one:
                self.assertEqual(keyframe, anim_one.animation_keyframes[key_idx])
            else:
                remapped_idx = key_idx - anim_one.keyframe_count
                original_keyframe = anim_two.animation_keyframes[remapped_idx]
                original_keyframe.time += final_time_one + delay
                self.assertEqual(keyframe, original_keyframe)

    def test_adjust_keyframe_to_timestamp(self) -> None:
        """
        Test remapping specific keyframes to new timestamps and interpolate the values inbetween
        """
        simple_gripper_builder_one = AnimationBuilder.from_animation(simple_gripper_animation())

        # Only remap the start and end of the sequence
        keyframe_idxs = [1, 2]
        timestamps_to_remap_one = [10, 15]

        adjust_keyframes_to_timestamps(simple_gripper_builder_one, keyframe_idxs, timestamps_to_remap_one, self._logger)

        new_timestamps = create_copy_of_timestamps(simple_gripper_builder_one)
        for timestamp_idx, keyframe_idx in enumerate(keyframe_idxs):
            self.assertEqual(new_timestamps[keyframe_idx], timestamps_to_remap_one[timestamp_idx])


if __name__ == "__main__":
    unittest.main()
