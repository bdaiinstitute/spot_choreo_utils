#  Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.
#
# Functions that modify the contents of a semantic animation to accomplish a specific task
# If a function is generally useful for keeping animations and their semantic data in sync
# it should be part of the SemanticAnimationBuilder class. If it's a specific way to modify
# a semantic animation it should go here

import logging
import unittest
from pathlib import Path

from test_animation_builder import simple_gripper_animation

from spot_choreo_utils.choreo_creation.choreo_builders.animation_builder import (
    AnimationBuilder,
    check_if_keyframe_poses_equivalent,
)
from spot_choreo_utils.choreo_creation.semantic_animations.semantic_animation_builder import SemanticAnimationBuilder
from spot_choreo_utils.choreo_creation.semantic_animations.semantic_animation_operators import (
    repeat_named_segment,
    reverse_named_segment,
)
from spot_choreo_utils.paths import get_example_choreo_path
from spot_choreo_utils.serialization.serialization_utils import load_animation


class SemanticAnimationTester(unittest.TestCase):
    def setUp(self) -> None:
        animation_path = Path(get_example_choreo_path(), "animations", "pose_to_pose_animation.pbtxt")
        self._animation = load_animation(animation_path)
        self._logger = logging.Logger("AnimationBuilderTestLogger")

    def test_repeat_named_segment(self) -> None:
        """
        Test repeating named segments
        """
        segment_one_name = "seg-1"
        segment_one_range = (0, 1)
        segment_two_name = "seg-2"
        insert_index = 2

        semantic_builder = SemanticAnimationBuilder()
        semantic_builder.start_from_animation(simple_gripper_animation())
        # Edit the first keyframe to make insertion point cleraer in loop
        semantic_builder.animation_builder._animation.animation_keyframes[0].gripper.gripper_angle.value = 1

        semantic_builder.name_segment_by_index(
            segment_name=segment_one_name, start_index=segment_one_range[0], end_index=segment_one_range[1]
        )

        repeat_named_segment(
            semantic_builder=semantic_builder,
            segment_name=segment_one_name,
            new_segment_name=segment_two_name,
            insertion_index=insert_index,
        )
        self.assertEqual(
            semantic_builder.get_segment_ranges(),
            {
                segment_one_name: segment_one_range,
                segment_two_name: (2, 3),
            },
        )

    def test_reverse_named_segment(self) -> None:
        """
        Test trepeating named segments
        """
        build_settings = AnimationBuilder.BuildSettings()
        build_settings.apply_unique_name = False

        segment_one_name = "seg-1"
        segment_one_range = (0, 1)

        semantic_builder = SemanticAnimationBuilder()
        semantic_builder.start_from_animation(simple_gripper_animation())
        # Edit the first keyframe to make insertion point cleraer in loop
        semantic_builder.animation_builder._animation.animation_keyframes[0].gripper.gripper_angle.value = 1

        starting_animation = semantic_builder.build(build_settings)

        semantic_builder.name_segment_by_index(
            segment_name=segment_one_name, start_index=segment_one_range[0], end_index=segment_one_range[1]
        )
        print(semantic_builder.animation_builder._animation)

        reverse_named_segment(semantic_builder=semantic_builder, segment_name=segment_one_name)
        modified_animation = semantic_builder.build(build_settings)
        self.assertIsNotNone(modified_animation)
        # Keyframes shouldn't match with updated times
        zero_original = starting_animation.animation_keyframes[0]
        one_original = starting_animation.animation_keyframes[1]
        zero_modified = modified_animation.animation_keyframes[0]
        one_modified = modified_animation.animation_keyframes[1]
        self.assertNotEqual(zero_original, one_modified)
        self.assertNotEqual(zero_modified, one_original)
        # Poses should match
        self.assertTrue(check_if_keyframe_poses_equivalent(zero_original, one_modified))
        self.assertTrue(check_if_keyframe_poses_equivalent(zero_modified, one_original))

        # A second reversal should restore the animation
        reverse_named_segment(semantic_builder=semantic_builder, segment_name=segment_one_name)
        modified_animation = semantic_builder.build(build_settings)
        self.assertEqual(starting_animation, modified_animation)


if __name__ == "__main__":
    unittest.main()
