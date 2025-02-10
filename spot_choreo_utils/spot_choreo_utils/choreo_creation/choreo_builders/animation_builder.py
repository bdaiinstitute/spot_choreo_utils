# Copyright (c) 2023-2025 Boston Dynamics AI Institute LLC. All rights reserved.

import copy
import math
import random
import string
from bisect import bisect_left
from logging import Logger
from typing import Dict, List, Optional, Tuple

from bosdyn.api.spot.choreography_sequence_pb2 import (
    AnimateArm,
    AnimateBody,
    AnimateGripper,
    AnimateLegs,
    Animation,
    AnimationKeyframe,
)

from spot_choreo_utils.choreo_creation.choreo_builders.animation_proto_utils import (
    build_gripper_params,
    check_if_keyframe_poses_equivalent,
    ensure_protobuf_compliance,
    joint_angle_keyframe_to_proto,
)


class AnimationBuilder:
    """Wrapper for building, editing, and validating choreography animations"""

    class BuildSettings:
        # Whether to apply a unique name suffix to the end of the animation
        # Animation uploads, validation and playback sometimes have unexpected behavior if
        # a previously uploaded animation has the same name, so when building an animation to
        # play back on robot, it's best practice to always assign it a unique name
        apply_unique_name: bool = True
        # Whether the builder should automatically remove keyframes set to the same timestamp
        remove_duplicate_timestamps: bool = False
        # Whether the builder should return an animation if it fails validation
        only_output_valid: bool = True
        # Choreography ends as soon as the final keyframe plays.
        # Unless the final pose is a default pose for the robot, it's often useful to
        # hold the final pose
        hold_final_pose_s: float = 0
        # Each keyframe must indicate whether each leg is in contact with the ground or not
        # Either manually specify leg contact status per-keyframe, or pass this argument
        # to automatically label all keyframes as stance on the ground
        apply_stance_to_all_keyframes: bool = True

    def __init__(self, logger: Optional[Logger] = None) -> None:
        if logger is None:
            logger = Logger("animation_builder")

        self._animation = Animation()
        self._logger: Logger = logger

    @classmethod
    def from_animation(cls, animation: Animation, logger: Optional[Logger] = None) -> "AnimationBuilder":
        """Transform animation into animation builder"""
        if logger is None:
            logger = Logger("animation_builder_logger")
        animation_builder = AnimationBuilder(logger)
        animation_builder.start_from_animation(animation)
        return animation_builder

    def start_from_empty(self, name: str) -> None:
        """Build an animation procedurally from scratch"""
        self._animation.name = name

    def start_from_animation(self, animation_proto: Animation) -> None:
        """Modify an existing animation with builder helper functions"""
        if animation_proto is None:
            self._logger.error("Passed None - can't start from animation")
            return
        self._animation = copy.deepcopy(animation_proto)
        # Get rid of BPM paramater - it just creates confusion between the
        # animation and sequence - use procedural adjustment utils instead
        self._animation.bpm = 0
        self._update_control_declarations()

    @property
    def name(self) -> str:
        """Get the animation name"""
        return self._animation.name

    @property
    def logger(self) -> Logger:
        """Returns the logger associated with the animation builder"""
        return self._logger

    @property
    def keyframe_count(self) -> int:
        """Returns the length of the animation in keyframes"""
        return len(self._animation.animation_keyframes)

    @property
    def keyframe_timestamps(self) -> List[float]:
        """Return a list of all keyframe timestamps"""
        return [keyframe.time for keyframe in self._animation.animation_keyframes]

    @property
    def animation_length_s(self) -> float:
        """Rerturns the length of the animation in seconds"""
        keyframes = self._animation.animation_keyframes

        if len(keyframes) == 0:
            return 0
        else:
            return keyframes[-1].time

    def keyframe_at_index(self, keyframe_idx: int) -> Optional[AnimationKeyframe]:
        """
        Returns the keyframe if the index is valid, otherwise returns None
        """
        keyframe_count = self.keyframe_count

        if keyframe_idx < 0 or keyframe_idx >= keyframe_count:
            if self._logger:
                self._logger.error(
                    f"Requested keyframe at index {keyframe_idx} but animation length is {keyframe_count}"
                )
            return None

        return self._animation.animation_keyframes[keyframe_idx]

    def keyframe_range(self, start_idx: int, end_idx: int) -> list[AnimationKeyframe]:
        """
        Returns a list of keyframes that fall in the start/end range
        """
        if start_idx > end_idx:
            self._logger.error(f"Start index must be lower than end index: {start_idx} > {end_idx}")
            return []
        range_start = max(start_idx, 0)
        range_end = min(end_idx, self.keyframe_count)

        return self._animation.animation_keyframes[range_start:range_end]

    def time_elapsed_between_keyframes(self, first_index: int, second_index: int) -> float:
        """
        Returns the amount of time that's passed between two keyframes
        """
        first_keyframe = self.keyframe_at_index(first_index)
        second_keyframe = self.keyframe_at_index(second_index)

        if first_keyframe is None or second_keyframe is None or (first_index > second_index):
            self._logger.error(
                f"Can't get time between keyframes when one doesn't exist. First index {first_index}, second index"
                f" {second_index}"
            )
            return 0

        return second_keyframe.time - first_keyframe.time

    @property
    def raw_keyframes(self) -> list[AnimationKeyframe]:
        """
        Returns a list of all keyframes
        """
        return self._animation.animation_keyframes

    @property
    def raw_animation(self) -> Animation:
        """Returns the animation mid-build without validation or adjustments"""
        return self._animation

    def set_name(self, name: str) -> None:
        """Change the animation's name"""
        self._animation.name = name

    def set_logger(self, logger: Logger) -> None:
        """Set the logger the animation builder should use"""
        self._logger = logger

    def build(self, build_settings: BuildSettings = BuildSettings()) -> Optional[Animation]:
        """
        Validate and return a protobuf representing the final animation.

        Returns None if the animation is invalid and would fail on upload to the robot
        """
        self._update_control_declarations()

        # Remove duplicate keyframes (according to robot's keyframe precision)
        self._update_timestamps_to_robot_precision()
        if build_settings.remove_duplicate_timestamps:
            self._remove_duplicate_timestamps()

        # Create new copy for procedural edits that would conflict with
        # future builder operations
        output_animation = copy.deepcopy(self._animation)

        if build_settings.apply_stance_to_all_keyframes:
            for keyframe in output_animation.animation_keyframes:
                self.add_stance_to_keyframe(keyframe)

        if build_settings.only_output_valid:
            res, msg = self.validate()
            if not res:
                self._logger.error(f"Failed to build animation: {msg}")
                return None

        if build_settings.hold_final_pose_s:
            last_keyframe = self._animation.animation_keyframes[-1]
            new_time = last_keyframe.time + build_settings.hold_final_pose_s
            self.insert_keyframe_at_time(animation_keyframe=last_keyframe, start_time=new_time)

        if build_settings.apply_unique_name:
            unique_suffix = "".join(random.choices(string.ascii_uppercase, k=10))
            output_animation.name = f"{output_animation.name}_{unique_suffix}"

        # Special case - the robot doesn't like single keyframe animations
        # so duplicate the keyframe to make a simple go to pose
        if build_settings.only_output_valid and self.keyframe_count == 1:
            single_keyframe = copy.deepcopy(output_animation.animation_keyframes[0])
            new_time = single_keyframe.time + 0.1
            self.insert_keyframe_at_time(single_keyframe, new_time)

        return output_animation

    def append_animation(self, other_animation: "AnimationBuilder", delay: float = 0.05) -> None:
        """
        Appends an animation with a delay before the first keyframe to smoothly
        transition poses between the animations.
        """
        final_timestamp_anim_one = self.keyframe_timestamps[-1] + delay

        # Copy keyframes so base builder isn't changed
        keyframe_copy = copy.deepcopy(other_animation.raw_keyframes)
        for frame in keyframe_copy:
            frame.time = frame.time + final_timestamp_anim_one

        # extend the first animation
        self.raw_keyframes.extend(keyframe_copy)

    ###
    # Existing Keyframes - Insert and Overwrite
    ###

    def insert_keyframe_at_index(
        self,
        animation_keyframe: AnimationKeyframe,
        insert_index: int,
        adjust_trailing_keyframes: bool = False,
    ) -> None:
        """
        Add a keyframe to the animation, inserted at index
          adjust_trailing_keyframes: preserves the time offset of all trailing keyframes after the insertion
             e.g. if times are [0, 1, 2, 3] and a new keyframe is inserted at index 1 with time 1.3,
                  adjust_trailing_keyframes would result in [0, 1.3, 2.3, 3.3, 4.3]
        """
        new_keyframe = copy.deepcopy(animation_keyframe)
        self._animation.animation_keyframes.insert(insert_index, new_keyframe)

        if adjust_trailing_keyframes:
            new_time_offset = self.time_elapsed_between_keyframes(insert_index - 1, insert_index)
            # For every keyframe after the one just inserted, adjust their times to preserve previous offsets
            for update_idx in range(insert_index + 1, self.keyframe_count):
                self._animation.animation_keyframes[update_idx].time += new_time_offset

    def insert_keyframes_at_index(
        self,
        animation_keyframes: list[AnimationKeyframe],
        insert_index: int,
        adjust_trailing_keyframes: bool = False,
        is_keyframe_timing_relative: bool = False,
    ) -> None:
        """
        Add a list of keyframes to the animation at the requested index
          adjust_trailing_keyframes: preserves the time offset of all trailing keyframes after the insertion
             e.g. if times are [0, 1, 2, 3] and a new keyframe is inserted at index 1 with time 1.3,
                  adjust_trailing_keyframes would result in [0, 1.3, 2.3, 3.3, 4.3]
          is_keyframe_timing_relative: the time of the current keyframe at insert_index will be treated as time 0
                  and all animation_keyframes will be inserted relative to that time
        """
        new_keyframes = copy.deepcopy(animation_keyframes)
        # Calculate all keyframe times before performing any insertions
        if is_keyframe_timing_relative:
            current_time_at_insertion = 0
            keyframe = self.keyframe_at_index(insert_index)
            if keyframe is not None:
                current_time_at_insertion = keyframe.time
        else:
            current_time_at_insertion = 0
        modified_keyframe_times = [keyframe.time + current_time_at_insertion for keyframe in new_keyframes]

        # Insert keyframes
        for relative_idx, keyframe in enumerate(new_keyframes):
            keyframe.time = modified_keyframe_times[relative_idx]
            self.insert_keyframe_at_index(
                animation_keyframe=keyframe,
                insert_index=insert_index + relative_idx,
                adjust_trailing_keyframes=adjust_trailing_keyframes,
            )

    def insert_keyframe_at_time(
        self, animation_keyframe: AnimationKeyframe, start_time: float, adjust_trailing_keyframes: bool = False
    ) -> None:
        """
        Add a keyframe to the animation at the requested start_time
        """
        keyframe = copy.deepcopy(animation_keyframe)
        keyframe.time = start_time

        (insertion_idx, prior_keyframe) = self.get_keyframe_before_timestamp(start_time)
        if prior_keyframe is not None and prior_keyframe.time == keyframe.time:
            self._logger.error(f"Can't insert keyframe at time {keyframe.time}, already exists. Overwrite instead")
            return None

        if insertion_idx == 0 and self.keyframe_count > 0 and keyframe.time == 0:
            self._logger.error("Can't have two keyframes at timestamp 0")
            return None

        self.insert_keyframe_at_index(
            animation_keyframe=keyframe,
            insert_index=insertion_idx + 1,
            adjust_trailing_keyframes=adjust_trailing_keyframes,
        )

    def overwrite_keyframe_at_index(
        self, animation_keyframe: AnimationKeyframe, keyframe_idx: int, match_time: bool = False
    ) -> None:
        """
        Replaces the keyframe at the requested index, optionally matching the current time
        """
        if match_time:
            keyframe = self.keyframe_at_index(keyframe_idx)
            if keyframe:
                keyframe_time = keyframe.time
                animation_keyframe.time = keyframe_time

        # Protobufs don't allow direct overwrites, so copy to array and back
        all_keyframes = self._animation.animation_keyframes[:]
        all_keyframes[keyframe_idx] = animation_keyframe
        del self._animation.animation_keyframes[:]
        self._animation.animation_keyframes.extend(all_keyframes)

    def insert_keyframe_from_dict(
        self, keyframe: Dict[str, float], start_time: Optional[float] = None, adjust_trailing_keyframes: bool = False
    ) -> None:
        """Insert a keyframe defined by the dictionary at the requested time"""
        keyframe_proto = joint_angle_keyframe_to_proto(keyframe, start_time)
        self.insert_keyframe_at_time(keyframe_proto, keyframe_proto.time, adjust_trailing_keyframes)

    def insert_keyframes_from_dicts(
        self, keyframes: List[Dict[str, float]], adjust_trailing_keyframes: bool = False
    ) -> None:
        """
        Transforms a list of keyframes represented as {joint_name:angle,joint_name...}
        into a protobuf keyframe and appends it to the animation
        Each keyfame must include a start_time key, and start times should be
        monotonically increasing.
        """
        for keyframe_angles in keyframes:
            if "start_time" not in keyframe_angles.keys():
                print("No start time specified for keyframe, skipping")
                continue
            self.insert_keyframe_from_dict(keyframe_angles, keyframe_angles["start_time"], adjust_trailing_keyframes)

    ###
    # Existing Keyframes - Find, Remove, Edit
    ###

    def lookup_index_for_keyframe(
        self, animation_keyframe: AnimationKeyframe, timestamps_should_match: bool = False
    ) -> int:
        """
        Returns the index that matches a keyframe's data.
        When timestamps_should_match is true, keyframe will only be returned for an exact match
        When timestamps_should_match is false the function will return the index closest to the animation's
        timestamp that has the same robot pose.
           - The search for the keyframe may take longer when timstamps should match is false
        Returns -1 if matching keyframe is not found
        """
        if animation_keyframe is None:
            return -1

        (candidate_idx, candidate_keyframe) = self.get_keyframe_before_timestamp(animation_keyframe.time)
        if candidate_keyframe is not None:
            is_exact_match = check_if_keyframe_poses_equivalent(animation_keyframe, candidate_keyframe)
        else:
            is_exact_match = False

        # Optionally, search in either direction from previous candidate index to find a keyframe pose match
        if not is_exact_match and not timestamps_should_match:
            # Search forward and backward simultaneously
            # It's safe to get indexes that are out of range with these function calls,
            # so take the larger value and search until reaching the end
            range_to_search = max(candidate_idx, self.keyframe_count - candidate_idx) + 1
            for offset in range(1, range_to_search):
                forward_idx = candidate_idx + offset
                backward_idx = candidate_idx - offset

                forward_keyframe = self.keyframe_at_index(forward_idx) if forward_idx < self.keyframe_count else None
                backward_keyframe = self.keyframe_at_index(backward_idx) if backward_idx >= 0 else None
                if forward_keyframe is not None and check_if_keyframe_poses_equivalent(
                    forward_keyframe, animation_keyframe
                ):
                    return forward_idx
                if backward_keyframe is not None and check_if_keyframe_poses_equivalent(
                    backward_keyframe, animation_keyframe
                ):
                    return backward_idx

        return candidate_idx if is_exact_match else -1

    def get_keyframe_before_timestamp(
        self,
        time: float,
    ) -> Tuple[int, Optional[AnimationKeyframe]]:
        """Returns the keyframe before the requested time"""
        all_timestamps = self.keyframe_timestamps

        insertion_order_idx = bisect_left(all_timestamps, time)
        if insertion_order_idx >= len(all_timestamps):
            keyframe_idx = insertion_order_idx - 1
        elif all_timestamps[insertion_order_idx] > time:
            keyframe_idx = max(insertion_order_idx - 1, 0)
        else:
            keyframe_idx = insertion_order_idx

        return (keyframe_idx, self.keyframe_at_index(keyframe_idx))

    def remove_keyframe_at_idx(self, keyframe_idx: int, adjust_trailing_keyframes: bool = False) -> None:
        """
        Remove the keyframe at the requested index, optionally moving all trailing keyframes up
        """
        num_keyframes = self.keyframe_count

        if num_keyframes == 0 or keyframe_idx >= num_keyframes:
            self._logger.error(
                f"Invalid keyframe index. Can't remove keyframe {keyframe_idx} from animation length {num_keyframes}"
            )
            return

        # Remove the time between this keyframe and the next keyframe
        if adjust_trailing_keyframes and keyframe_idx < self.keyframe_count - 1:
            if not self.keyframe_at_index(keyframe_idx):
                self._logger.error(f"Error accessing keyframe at index {str(keyframe_idx)}")
                return
            if not self.keyframe_at_index(keyframe_idx + 1):
                self._logger.error(f"Error accessing keyframe at index {str(keyframe_idx + 1)}")
                return
            time_between_keyframes = self.time_elapsed_between_keyframes(keyframe_idx, keyframe_idx + 1)
            for index_to_modify in range(keyframe_idx + 1, self.keyframe_count):
                modify_keyframe = self.keyframe_at_index(index_to_modify)
                if not modify_keyframe:
                    self._logger.error(f"Error accessing keyframe at index {str(index_to_modify)}")
                    return
                modify_keyframe.time -= time_between_keyframes

        # Remove the keyframe
        del self._animation.animation_keyframes[keyframe_idx]

    def change_keyframe_time_at_index(self, keyframe_index: int, new_start_time: float) -> None:
        """Adjust the start time for a keyframe"""
        keyframe = self.keyframe_at_index(keyframe_index)
        if keyframe is None:
            self._logger.error(f"No keyframe at index {keyframe_index}")
            return
        keyframe.time = new_start_time

    ###
    # New Keyframes - Build programatically
    ###

    def animate_gripper(self, start_time: float, gripper_angle: float) -> None:
        """Create a gripper keyframe"""
        gripper_angle = ensure_protobuf_compliance(gripper_angle, adjust_positive=False)

        if gripper_angle < -1 or gripper_angle > 0:
            self._logger.error("INVALID GRIPPER ANGLE - MUST BE IN RANGE -1 to 0")
            return

        self.create_keyframe_from_params(start_time=start_time, gripper=build_gripper_params(gripper_angle))
        self._animation.controls_gripper = True

    def create_keyframe_from_params(
        self,
        start_time: float,
        gripper: AnimateGripper = None,
        arm: AnimateArm = None,
        body: AnimateBody = None,
        legs: AnimateLegs = None,
    ) -> None:
        """Add a keyframe to the animation with paramaterized protobufs"""

        new_keyframe = AnimationKeyframe()
        new_keyframe.time = start_time
        if gripper:
            new_keyframe.gripper.CopyFrom(gripper)
        if arm:
            new_keyframe.arm.CopyFrom(arm)
        if body:
            new_keyframe.body.CopyFrom(body)
        if legs:
            new_keyframe.legs.CopyFrom(legs)

        self.insert_keyframe_at_time(new_keyframe, new_keyframe.time)

    ###
    # Validation and automatic animation fixes
    ###
    def validate(self) -> Tuple[bool, str]:
        """Offline validator that matches reasons animations fail to upload to robot"""
        if not self._animation.name:
            return False, "Animation does not have a name"

        if not self._animation.animation_keyframes:
            return False, "Animation is empty. Add at least one keyframe"

        # First Keyframe must start at time 0 to guarantee consistent playback
        first_keyframe_time = self._animation.animation_keyframes[0].time
        if not math.isclose(first_keyframe_time, 0, abs_tol=1e-6):
            return False, f"First keyframe must be at time 0, not {first_keyframe_time}"

        # Check all keyframes for sequence requirements
        last_time_seen = -1
        for idx, keyframe in enumerate(self._animation.animation_keyframes):
            if keyframe.time <= last_time_seen:
                return False, f"Keyframe timestamps must be monotonically increasing. Error at keyframe {idx}"
            last_time_seen = keyframe.time

            if keyframe.HasField("gripper") and not self._animation.controls_gripper:
                return False, "Animation controls gripper, but controls_gripper paramater not set"
            if keyframe.HasField("arm") and not self._animation.controls_arm:
                return False, "Animation controls arm, but controls_arm paramater not set"
            if keyframe.HasField("body") and not self._animation.controls_body:
                return False, "Animation controls the body, but controls_body paramater not set"
            if keyframe.HasField("legs") and not self._animation.controls_legs:
                return False, "Animation controls the legs, but controls_legs paramater not set"
        return True, "success"

    def add_stance_to_keyframe(self, keyframe: AnimationKeyframe) -> None:
        """
        Set the stance value for all legs in a keyframe
        """
        if keyframe.HasField("legs"):
            keyframe.legs.fl.stance.value = True
            keyframe.legs.fr.stance.value = True
            keyframe.legs.hl.stance.value = True
            keyframe.legs.hr.stance.value = True

    def _update_control_declarations(self) -> None:
        """
        Animations must specify which parts of the robot they control.
        This function ensures that the control statements match the animation contents.
        """
        self._animation.controls_gripper = False
        self._animation.controls_arm = False
        self._animation.controls_body = False
        self._animation.controls_legs = False

        for keyframe in self._animation.animation_keyframes:
            if keyframe.HasField("gripper"):
                self._animation.controls_gripper = True
            if keyframe.HasField("arm"):
                self._animation.controls_arm = True
            if keyframe.HasField("body"):
                self._animation.controls_body = True
            if keyframe.HasField("legs"):
                self._animation.controls_legs = True

    def _remove_duplicate_timestamps(self) -> None:
        """
        Removes duplicate timestamps in the sequence. Given the timestamp precision
        on the robot, procedural modifications may result in exact matches
        which must be removed
        """
        known_timestamps = set()
        # The add is meaningless as a truth check - just lets us modify set within the comprehension
        duplicates = [
            idx
            for (idx, x) in enumerate(self._animation.animation_keyframes)
            if x.time in known_timestamps or known_timestamps.add(x.time)  # type: ignore
        ]
        for dup_idx in reversed(duplicates):
            del self._animation.animation_keyframes[dup_idx : dup_idx + 1]

    def _update_timestamps_to_robot_precision(self) -> None:
        """Match the time precision on robot"""
        for keyframe in self._animation.animation_keyframes:
            keyframe.time = float(f"{keyframe.time:.4f}")
