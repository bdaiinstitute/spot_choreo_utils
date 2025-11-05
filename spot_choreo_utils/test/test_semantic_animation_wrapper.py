#  Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.
import copy
import tempfile
import unittest
from pathlib import Path

from test_animation_builder import simple_gripper_animation

import spot_choreo_utils.serialization.serialization_utils as serialization_utils
from spot_choreo_utils.choreo_creation.choreo_builders.animation_builder import AnimationBuilder
from spot_choreo_utils.choreo_creation.semantic_animations.semantic_animation_builder import SemanticAnimationBuilder
from spot_choreo_utils.paths import get_example_choreo_path


class SemanticAnimationBuilderTester(unittest.TestCase):
    def setUp(self) -> None:
        Path(
            get_example_choreo_path(),
        ).resolve()
        self.test_csq_path = Path(get_example_choreo_path(), "sequences", "pose_to_pose_sequence.pbtxt").resolve()
        self.test_cha_path = Path(get_example_choreo_path(), "animations", "pose_to_pose_animation.pbtxt").resolve()

    def test_load_animation(self) -> None:
        """Load an animation files in different ways from disk"""
        build_settings = AnimationBuilder.BuildSettings()
        build_settings.apply_unique_name = False

        animation_proto = serialization_utils.load_animation(self.test_cha_path)
        self.assertIsNotNone(animation_proto)

        # Pass the animation in directly
        semantic_animation_pass_in = SemanticAnimationBuilder()
        semantic_animation_pass_in.start_from_animation(animation_proto)

        self.assertTrue(semantic_animation_pass_in.validate())
        self.assertIsNotNone(semantic_animation_pass_in.animation_builder)

        # Load the animation by file path and make sure it matches the one loaded directly
        semantic_animation_loaded = SemanticAnimationBuilder()
        semantic_animation_loaded.load_animation(self.test_cha_path)
        self.assertIsNotNone(semantic_animation_loaded.animation_builder)

        self.assertTrue(semantic_animation_loaded.validate())
        self.assertEqual(
            semantic_animation_pass_in.build(build_settings),
            semantic_animation_loaded.build(build_settings),
        )

    def test_load_save_semantic_animation(self) -> None:
        """Load and save semantic animations from disk"""
        build_settings = AnimationBuilder.BuildSettings()
        build_settings.apply_unique_name = False

        semantic_animation_original = SemanticAnimationBuilder()
        semantic_animation_original.load_animation(self.test_cha_path)

        with tempfile.TemporaryDirectory() as temp_dir:
            saved_path = semantic_animation_original.save_semantic_animation(temp_dir)
            self.assertIsNotNone(saved_path)
            semantic_animation_reloaded = SemanticAnimationBuilder()
            semantic_animation_reloaded.load_semantic_animation(saved_path)
            self.assertEqual(
                semantic_animation_original._animation_builder.build(build_settings),
                semantic_animation_reloaded._animation_builder.build(build_settings),
            )
            self.assertEqual(
                semantic_animation_original._semantic_metadata, semantic_animation_reloaded._semantic_metadata
            )

    def test_edit_animation(self) -> None:
        """Test functions that edit the animation"""
        build_settings = AnimationBuilder.BuildSettings()
        build_settings.apply_unique_name = False
        build_settings.apply_stance_to_all_keyframes = False

        complex_animation = serialization_utils.load_animation(self.test_cha_path)

        # reset the animation wrapper
        semantic_builder = SemanticAnimationBuilder()
        semantic_builder.start_from_animation(simple_gripper_animation())

        keyframe_to_insert = copy.deepcopy(complex_animation.animation_keyframes[0])
        keyframe_to_insert.time = 0.5

        semantic_builder.insert_keyframe_at_time(
            animation_keyframe=keyframe_to_insert, start_time=keyframe_to_insert.time
        )

        new_first_keyframe = semantic_builder.build(build_settings).animation_keyframes[1]
        print(new_first_keyframe)
        print(keyframe_to_insert)
        self.assertEqual(new_first_keyframe, keyframe_to_insert)

        # reset the animation wrapper
        semantic_builder = SemanticAnimationBuilder()
        semantic_builder.start_from_animation(simple_gripper_animation())

        keyframe_to_insert.time = 100

        semantic_builder.insert_keyframe_at_time(
            animation_keyframe=keyframe_to_insert, start_time=keyframe_to_insert.time
        )
        new_last_keyframe = semantic_builder.build(build_settings).animation_keyframes[-1]
        self.assertEqual(new_last_keyframe, keyframe_to_insert)

        # Now remove a keyframe - default for simple girpper is a time spacing of 1
        new_final_time = keyframe_to_insert.time - 1
        semantic_builder.remove_keyframe_at_idx(keyframe_idx=0, adjust_trailing_keyframes=True)
        new_last_keyframe = semantic_builder.build(build_settings).animation_keyframes[-1]
        self.assertEqual(new_last_keyframe.time, new_final_time)
        # Now the time of the last keyframe should stay the same without the adjustment
        semantic_builder.remove_keyframe_at_idx(keyframe_idx=1, adjust_trailing_keyframes=False)
        new_last_keyframe = semantic_builder.build(build_settings).animation_keyframes[-1]
        self.assertEqual(new_last_keyframe.time, new_final_time)

        # Make sure nothing weird happens when we remove past the number of keyframes
        semantic_builder.remove_keyframe_at_idx(keyframe_idx=0, adjust_trailing_keyframes=False)
        semantic_builder.remove_keyframe_at_idx(keyframe_idx=5, adjust_trailing_keyframes=False)
        semantic_builder.remove_keyframe_at_idx(keyframe_idx=-1, adjust_trailing_keyframes=False)
        semantic_builder.remove_keyframe_at_idx(keyframe_idx=0, adjust_trailing_keyframes=False)

    def test_semantic_naming(self) -> None:
        """Test functions that add and remove semantic names"""

        empty_dictionary: dict = {}
        arbitrary_segment_name = "A7-WhirlyBit"

        semantic_builder = SemanticAnimationBuilder()
        semantic_builder.start_from_animation(simple_gripper_animation())

        # Try to name segments with invalid paramaters
        semantic_builder.name_segment_by_index(segment_name=arbitrary_segment_name, start_index=-1, end_index=1)
        self.assertEqual(semantic_builder.get_segment_ranges(), empty_dictionary)
        self.assertIsNone(semantic_builder.get_segment_range(arbitrary_segment_name))

        semantic_builder.name_segment_by_index(segment_name=arbitrary_segment_name, start_index=0, end_index=5)
        self.assertEqual(semantic_builder.get_segment_ranges(), empty_dictionary)
        self.assertIsNone(semantic_builder.get_segment_range(arbitrary_segment_name))

        # Successfully name a segment
        segment_range = (0, 0)
        semantic_builder.name_segment_by_index(segment_name=arbitrary_segment_name, start_index=0, end_index=0)
        self.assertEqual(semantic_builder.get_segment_ranges(), {arbitrary_segment_name: segment_range})
        self.assertEqual(semantic_builder.get_segment_range(arbitrary_segment_name), segment_range)

        # Remove the segment to reset
        semantic_builder.remove_segment_name(segment_name=arbitrary_segment_name)
        self.assertEqual(semantic_builder.get_segment_ranges(), empty_dictionary)

        segment_range = (0, 2)
        semantic_builder.name_segment_by_index(segment_name=arbitrary_segment_name, start_index=0, end_index=2)
        self.assertEqual(semantic_builder.get_segment_ranges(), {arbitrary_segment_name: segment_range})
        self.assertEqual(semantic_builder.get_segment_range(arbitrary_segment_name), segment_range)

        # Try to add second segment with same name
        semantic_builder.name_segment_by_index(segment_name=arbitrary_segment_name, start_index=1, end_index=2)
        self.assertEqual(semantic_builder.get_segment_ranges(), {arbitrary_segment_name: segment_range})

        # Try to add overlapping segment
        semantic_builder.name_segment_by_index(segment_name="hmm", start_index=1, end_index=2)
        self.assertEqual(semantic_builder.get_segment_ranges(), {arbitrary_segment_name: segment_range})

        # Remove segment names that don't exist and invalid arguments
        semantic_builder.remove_segment_name(segment_name="nope")
        semantic_builder.remove_segment_name(segment_name="")
        self.assertEqual(semantic_builder.get_segment_ranges(), {arbitrary_segment_name: segment_range})

    def test_semantic_edits(self) -> None:
        """Test functions that edit the animation with semantic names"""
        build_settings = AnimationBuilder.BuildSettings()
        build_settings.apply_unique_name = False

        segment_one_name = "seg-1"
        segment_two_name = "seg-2"
        segment_three_name = "seg-3"
        segment_one_range = (0, 0)
        segment_two_range = (1, 1)
        segment_three_range = (2, 2)

        def setup_three_named_segments() -> SemanticAnimationBuilder:
            nonlocal segment_one_name, segment_two_name, segment_three_name
            nonlocal segment_one_range, segment_two_range, segment_three_range

            semantic_builder = SemanticAnimationBuilder()
            semantic_builder.start_from_animation(simple_gripper_animation())

            semantic_builder.name_segment_by_index(
                segment_name=segment_one_name, start_index=segment_one_range[0], end_index=segment_one_range[1]
            )
            semantic_builder.name_segment_by_index(
                segment_name=segment_two_name, start_index=segment_two_range[0], end_index=segment_two_range[1]
            )
            semantic_builder.name_segment_by_index(
                segment_name=segment_three_name, start_index=segment_three_range[0], end_index=segment_three_range[1]
            )
            return semantic_builder

        # Delete the first segment without adjusting the times
        semantic_builder = setup_three_named_segments()
        self.assertEqual(
            semantic_builder.get_segment_ranges(),
            {
                segment_one_name: segment_one_range,
                segment_two_name: segment_two_range,
                segment_three_name: segment_three_range,
            },
        )

        adjusted_segment_two_range = (segment_two_range[0] - 1, segment_two_range[1] - 1)
        adjusted_segment_three_range = (segment_three_range[0] - 1, segment_three_range[1] - 1)
        semantic_builder.delete_animation_segment(segment_one_name, adjust_trailing_keyframes=False)
        self.assertEqual(
            semantic_builder.get_segment_ranges(),
            {segment_two_name: adjusted_segment_two_range, segment_three_name: adjusted_segment_three_range},
        )
        # Since times weren't adjusted aniamtion doesn't have a 0 timestamp and this call returns none
        self.assertIsNone(semantic_builder.build(build_settings))
        # Check the timestamp explicitly
        self.assertEqual(semantic_builder.animation_builder._animation.animation_keyframes[0].time, 1)

        # Reset, delete the first segment with adjusting the times
        semantic_builder = setup_three_named_segments()

        adjusted_segment_two_range = (segment_two_range[0] - 1, segment_two_range[1] - 1)
        adjusted_segment_three_range = (segment_three_range[0] - 1, segment_three_range[1] - 1)
        semantic_builder.delete_animation_segment(segment_one_name, adjust_trailing_keyframes=True)

        self.assertEqual(
            semantic_builder.get_segment_ranges(),
            {segment_two_name: adjusted_segment_two_range, segment_three_name: adjusted_segment_three_range},
        )
        # Since times were adjusted the animation should be valid
        self.assertIsNotNone(semantic_builder.build(build_settings))


if __name__ == "__main__":
    unittest.main()
