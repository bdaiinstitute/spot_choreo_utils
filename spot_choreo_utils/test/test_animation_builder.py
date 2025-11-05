#  Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.
import copy
import logging
import unittest
from pathlib import Path

from bosdyn.api.spot.choreography_sequence_pb2 import (
    AnimateGripper,
    Animation,
    AnimationKeyframe,
)
from google.protobuf.wrappers_pb2 import DoubleValue

from spot_choreo_utils.choreo_creation.choreo_builders.animation_builder import AnimationBuilder
from spot_choreo_utils.choreo_creation.choreo_builders.animation_proto_utils import check_if_protobuf_field_set
from spot_choreo_utils.paths import get_example_choreo_path
from spot_choreo_utils.serialization.serialization_utils import load_animation

GRIPPER_ANIMATION_NAME = "gripper_open_close"


def simple_gripper_animation() -> Animation:
    # Define joint states
    GRIPPER_OPEN_ANGLE = -1
    GRIPPER_CLOSED_ANGLE = 0

    gripper_open = AnimateGripper()
    gripper_open.gripper_angle.CopyFrom(DoubleValue(value=GRIPPER_OPEN_ANGLE))

    gripper_close = AnimateGripper()
    gripper_close.gripper_angle.CopyFrom(DoubleValue(value=GRIPPER_CLOSED_ANGLE))

    # Create keyframe by combining joint states with timestamps
    start_keyframe = AnimationKeyframe()
    start_keyframe.time = 0
    start_keyframe.gripper.CopyFrom(gripper_close)

    mid_keyframe = AnimationKeyframe()
    mid_keyframe.time = 1
    mid_keyframe.gripper.CopyFrom(gripper_open)

    end_keyframe = AnimationKeyframe()
    end_keyframe.time = 2
    end_keyframe.gripper.CopyFrom(gripper_close)

    # Add keyframes to an animation
    gripper_animation_proto = Animation()
    gripper_animation_proto.name = GRIPPER_ANIMATION_NAME

    gripper_animation_proto.controls_gripper = True
    gripper_animation_proto.animation_keyframes.append(start_keyframe)
    gripper_animation_proto.animation_keyframes.append(mid_keyframe)
    gripper_animation_proto.animation_keyframes.append(end_keyframe)

    return gripper_animation_proto


def create_clean_simple_animation_builder() -> AnimationBuilder:
    """
    Creates an animation builder with the simple gripper animation
    """
    simple_builder = AnimationBuilder.from_animation(simple_gripper_animation())
    simple_builder._update_timestamps_to_robot_precision()
    return simple_builder


class AnimationBuilderTester(unittest.TestCase):
    def setUp(self) -> None:
        animation_path = Path(get_example_choreo_path(), "animations", "pose_to_pose_animation.pbtxt")
        self._whole_body_animation_builder = AnimationBuilder.from_animation(load_animation(animation_path))
        self._simple_animation_builder = create_clean_simple_animation_builder()
        self._whole_body_animation_builder._update_timestamps_to_robot_precision()
        self._logger = logging.Logger("AnimationBuilderTestLogger")

    def test_proto_field_set(self) -> None:
        """Test check that a field is set within a protobuf"""
        animation = Animation()
        self.assertFalse(check_if_protobuf_field_set(animation, "name"))
        animation.name = "Test Name"
        self.assertTrue(check_if_protobuf_field_set(animation, "name"))

    def test_get_keyframe_functions(self) -> None:
        """
        Test the different ways to find a keyframe
        """
        simple_anim_length = self._simple_animation_builder.keyframe_count
        simple_animation = self._simple_animation_builder._animation
        complex_anim_length = self._whole_body_animation_builder.keyframe_count
        complex_animation = self._whole_body_animation_builder._animation

        ####
        # Get a single keyframe
        ####
        self.assertIsNone(self._simple_animation_builder.keyframe_at_index(-1))
        self.assertIsNone(self._simple_animation_builder.keyframe_at_index(simple_anim_length))
        index_to_check = 2
        self.assertEqual(
            self._simple_animation_builder.keyframe_at_index(index_to_check),
            simple_animation.animation_keyframes[index_to_check],
        )

        ####
        # Get a range of keyframes
        ####
        start_range = 3
        end_range = 5
        keyframe_true_range = self._whole_body_animation_builder.keyframe_range(start_range, end_range)
        keyframe_true_range_direct_access = complex_animation.animation_keyframes[start_range:end_range]
        self.assertListEqual(keyframe_true_range, keyframe_true_range_direct_access)

        # Cuts off at zero and goes to end range
        keyframe_extended_range = self._whole_body_animation_builder.keyframe_range(-20, end_range)
        keyframe_extended_range_direct_access = complex_animation.animation_keyframes[0:end_range]
        self.assertListEqual(keyframe_extended_range, keyframe_extended_range_direct_access)

        # Cuts at the end of the animation length appropriately
        keyframe_to_end = self._whole_body_animation_builder.keyframe_range(start_range, complex_anim_length + 20)
        self.assertEqual(keyframe_to_end[-1], complex_animation.animation_keyframes[-1])

        ####
        # Look up an index based on keyframe contents
        ####

        # Exact match
        exact_match_idx = 0
        keyframe_at_index = self._whole_body_animation_builder.keyframe_at_index(exact_match_idx)
        found_index = self._whole_body_animation_builder.lookup_index_for_keyframe(
            animation_keyframe=keyframe_at_index, timestamps_should_match=True
        )
        self.assertEqual(exact_match_idx, found_index)

        exact_match_idx = complex_anim_length - 1
        keyframe_at_index = self._whole_body_animation_builder.keyframe_at_index(exact_match_idx)
        found_index = self._whole_body_animation_builder.lookup_index_for_keyframe(
            animation_keyframe=keyframe_at_index, timestamps_should_match=True
        )
        self.assertEqual(exact_match_idx, found_index)

        exact_match_idx = 5
        keyframe_at_index = self._whole_body_animation_builder.keyframe_at_index(exact_match_idx)
        found_index = self._whole_body_animation_builder.lookup_index_for_keyframe(
            animation_keyframe=keyframe_at_index, timestamps_should_match=True
        )
        self.assertEqual(exact_match_idx, found_index)

        late_exact_match = complex_anim_length - 2
        keyframe_at_index = self._whole_body_animation_builder.keyframe_at_index(late_exact_match)
        found_index = self._whole_body_animation_builder.lookup_index_for_keyframe(
            animation_keyframe=keyframe_at_index, timestamps_should_match=True
        )
        self.assertEqual(late_exact_match, found_index)

        # Look up with time mismatch - should fail
        keyframe_at_index = self._whole_body_animation_builder.keyframe_at_index(late_exact_match)
        keyframe_at_index.time = 0
        found_index = self._whole_body_animation_builder.lookup_index_for_keyframe(
            animation_keyframe=keyframe_at_index, timestamps_should_match=True
        )
        self.assertEqual(found_index, -1)

        # Now let the function search and find the keyframe at an alternate index from timestamp
        found_index = self._whole_body_animation_builder.lookup_index_for_keyframe(
            animation_keyframe=keyframe_at_index, timestamps_should_match=False
        )
        self.assertEqual(found_index, late_exact_match)

    def test_overwrite_keyframe(self) -> None:
        """
        Test the different ways to overwrite keyframes
        """

        ####
        # Overwrite the keyframe - pull from
        ####

        complex_source_idx = 2
        complex_source_keyframe = self._whole_body_animation_builder.keyframe_at_index(complex_source_idx)

        # Overwrite but use the timestamp of the source keyframe
        index_to_overwrite = 0
        self._simple_animation_builder.overwrite_keyframe_at_index(
            keyframe_idx=index_to_overwrite, animation_keyframe=complex_source_keyframe
        )

        new_keyframe_time = self._simple_animation_builder.keyframe_at_index(index_to_overwrite).time
        self.assertEqual(complex_source_keyframe.time, new_keyframe_time)

    def test_insert_keyframe_at_index(self) -> None:
        """
        Test the different ways to insert keyframes
        """

        ####
        # insert_keyframe_at_index
        ####

        complex_source_idx = 2
        complex_source_keyframe = self._whole_body_animation_builder.keyframe_at_index(complex_source_idx)

        # Insert but use the timestamp of the source keyframe
        index_to_insert = 0
        self._simple_animation_builder.insert_keyframe_at_index(
            insert_index=index_to_insert,
            animation_keyframe=complex_source_keyframe,
            adjust_trailing_keyframes=False,
        )

        new_keyframe_time = self._simple_animation_builder.keyframe_at_index(index_to_insert).time
        self.assertEqual(complex_source_keyframe.time, new_keyframe_time)

        # reset for next test
        self._simple_animation_builder = create_clean_simple_animation_builder()

        # reset for next test
        self._simple_animation_builder = create_clean_simple_animation_builder()

        ##
        # Adjust the trailing keyframes based on the timestamp
        ##
        index_to_insert = 1
        time_to_overwrite_keyframe = self._simple_animation_builder.time_elapsed_between_keyframes(
            index_to_insert - 1, index_to_insert
        )
        self._simple_animation_builder.keyframe_at_index(index_to_insert).time
        self._simple_animation_builder.insert_keyframe_at_index(
            insert_index=index_to_insert,
            animation_keyframe=complex_source_keyframe,
            adjust_trailing_keyframes=True,
        )

        new_keyframe_time = self._simple_animation_builder.keyframe_at_index(index_to_insert).time
        moved_keyframe_time = self._simple_animation_builder.keyframe_at_index(index_to_insert + 1).time

        self.assertEqual(moved_keyframe_time, new_keyframe_time + time_to_overwrite_keyframe)

    def test_insert_keyframes_at_index(self) -> None:
        """
        Test inserting multiple keyframes into the animation
        """

        ####
        # insert_keyframes_at_index
        ####

        complex_source_idx = 2
        complex_end_idx = complex_source_idx + 4
        complex_source_keyframes = self._whole_body_animation_builder.raw_keyframes[complex_source_idx:complex_end_idx]
        preserved_source_keyframes = copy.deepcopy(complex_source_keyframes)

        original_animation_length = self._simple_animation_builder.keyframe_count
        insertion_index = 1
        insertion_index + len(complex_source_keyframes)
        self._simple_animation_builder.insert_keyframes_at_index(
            insert_index=insertion_index,
            animation_keyframes=complex_source_keyframes,
            adjust_trailing_keyframes=False,
        )
        self.assertEqual(complex_source_keyframes, preserved_source_keyframes)
        self.assertEqual(
            self._simple_animation_builder.keyframe_count,
            original_animation_length + len(complex_source_keyframes),
        )
        extracted_from_animation = self._simple_animation_builder._animation.animation_keyframes[
            insertion_index : insertion_index + len(complex_source_keyframes)
        ]
        self.assertListEqual(preserved_source_keyframes, extracted_from_animation)

    def test_remove_keyframe_at_index(self) -> None:
        """
        Test removing keyframes from the animation
        """

        ####
        # remove_keyframe_at_idx
        ####
        remove_idx = 1
        original_length = self._simple_animation_builder.keyframe_count
        self._simple_animation_builder.remove_keyframe_at_idx(keyframe_idx=remove_idx, adjust_trailing_keyframes=False)
        self.assertEqual(original_length - 1, self._simple_animation_builder.keyframe_count)

        # reset for next test
        self._simple_animation_builder = create_clean_simple_animation_builder()

        # Adjust trailing keyframes
        remove_idx = 1
        original_length = self._simple_animation_builder.keyframe_count
        self._simple_animation_builder.remove_keyframe_at_idx(keyframe_idx=remove_idx, adjust_trailing_keyframes=True)
        self.assertEqual(original_length - 1, self._simple_animation_builder.keyframe_count)
        final_time = self._simple_animation_builder.keyframe_at_index(1).time
        self.assertEqual(final_time, 1)

    def test_animation_information(self) -> None:
        """
        Test asking questions about a sequence
        """

        current_count = self._whole_body_animation_builder.keyframe_count
        current_direct_measure = len(self._whole_body_animation_builder._animation.animation_keyframes)
        self.assertEqual(current_count, current_direct_measure)

        time_elapsed = self._whole_body_animation_builder.time_elapsed_between_keyframes(-1, 50)
        self.assertEqual(time_elapsed, 0)

        time_elapsed = self._whole_body_animation_builder.time_elapsed_between_keyframes(50, current_count + 1)
        self.assertEqual(time_elapsed, 0)

        time_elapsed = self._whole_body_animation_builder.time_elapsed_between_keyframes(50, 50)
        self.assertEqual(time_elapsed, 0)


if __name__ == "__main__":
    unittest.main()
