# Copyright (c) 2023-2025 Boston Dynamics AI Institute LLC. All rights reserved.

import copy
from logging import Logger
from typing import List, Optional, Tuple, Union

import numpy as np
from bosdyn.api.spot.choreography_sequence_pb2 import Animation, AnimationKeyframe, ChoreographySequence

from spot_choreo_utils.choreo_creation.choreo_builders.animation_builder import (
    AnimationBuilder,
)
from spot_choreo_utils.choreo_creation.choreo_builders.animation_proto_utils import check_if_protobuf_field_set
from spot_choreo_utils.choreo_creation.choreo_builders.sequence_builder import SequenceBuilder
from spot_choreo_utils.choreo_creation.choreo_builders.spot_properties import joint_angle_to_protobuf_attrs_map


def create_single_animation_sequence(
    animation: Union[Animation, AnimationBuilder],
    build_settings: AnimationBuilder.BuildSettings,
    logger: Optional[Logger] = None,
) -> Tuple[Optional[Animation], Optional[ChoreographySequence]]:
    """
    Create a sequence and animation based on the builder that can play back on robot.
    If animation in builder is invalid returns (None, None)
    """
    if not logger:
        logger = Logger("animation_sequence_builder")

    if isinstance(animation, AnimationBuilder):
        animation = animation.build(build_settings)

    if animation is None:
        logger.error("Issue building animation")
        return (None, None)

    sequence_builder = SequenceBuilder(logger)
    sequence_builder.start_from_empty(name=animation.name, slices_per_minute=120)
    sequence_builder.add_animation(animation, start_time=0)
    sequence = sequence_builder.build()
    if sequence is None:
        logger.error("Issue building sequence")
        return (None, None)
    return (animation, sequence)


def find_last_standing_keyframe(builder: AnimationBuilder) -> int:
    """
    Animations must end with all legs marked as standing. Returns the last keyframe where this is
    true so that animations can be proecedurally trimmed for validity.
    """
    animation_length = builder.keyframe_count
    if animation_length > 0 and not check_if_protobuf_field_set(builder.raw_keyframes[0], "legs"):
        # The animation doesn't involve legs, so the last keyframe is valid
        return animation_length - 1

    for i in reversed(range(animation_length)):
        legs = builder.raw_keyframes[i].legs
        is_stance = legs.fl.stance.value
        is_stance &= legs.fr.stance.value
        is_stance &= legs.hl.stance.value
        is_stance &= legs.hr.stance.value
        if is_stance:
            return i
    return -1


def ensure_animation_ends_standing_stable(builder: AnimationBuilder) -> None:
    """
    If the animation doesn't end with all legs on the ground the end of the animation
    is cut off to the point that all legs are stable on the ground.
    If the animation doesn't control the legs then the robot is guaranteed to be standing stable
    """
    if builder.keyframe_count > 0 and check_if_protobuf_field_set(builder.raw_keyframes[0], "legs"):
        last_standing_keyframe = find_last_standing_keyframe(builder)
        for idx in reversed(range(last_standing_keyframe + 1, builder.keyframe_count)):
            builder.remove_keyframe_at_idx(idx)


def adjust_keyframes_to_timestamps(
    animation_builder: AnimationBuilder,
    keyframe_idxs: List[int],
    new_timestamps: List[float],
    logger: Optional[Logger] = None,
) -> None:
    """
    For each keyframe in the list, adjust its timestamp to align with the provided timestamp list
    All intermediate keyframes will be propotionially adjusted to match the new timestamps.
    E.G. starting with timestamps [0, 1, 2, 3, 4] and adjusting the 1st and 4th index to 2, 10
    stretches the animation proportionaitly [0, 2, 4.66, 7.32, 10]
    """
    if len(keyframe_idxs) != len(new_timestamps):
        if logger is not None:
            logger.error("Can't adjust timestamps. Index and timestamps must be of same length.")
        return

    keyframe_monotonic = np.all(np.diff(keyframe_idxs) > 0)
    timestamp_monotonic = np.all(np.diff(new_timestamps) > 0)

    if not keyframe_monotonic or not timestamp_monotonic:
        if logger is not None:
            logger.error("Times and indexes must be monotonically increasing for proper interpolation.")
        return

    # Build up a new list of keyframes with adjusted timestamps
    updated_keyframes = [animation_builder.raw_keyframes[0]]

    # Walk through each of the requested updates, adjust its time, and then interpolate
    # all keyframes inbetween
    for update_idx in range(len(keyframe_idxs)):
        # Anchor keyframe stays at current time, adjust keyframe moves to new time
        anchor_keyframe_idx = keyframe_idxs[update_idx - 1] if update_idx > 0 else 0
        adjust_keyframe_idx = keyframe_idxs[update_idx]
        adjust_target_time = new_timestamps[update_idx]

        source_anchor_keyframe = animation_builder.raw_keyframes[anchor_keyframe_idx]
        source_adjust_keyframe = animation_builder.raw_keyframes[adjust_keyframe_idx]

        # Extract the intervals of the source sequence vs new sequence timing.
        # This ratio will determine the distribution of interpolated keyframes
        last_updated_keyframe = updated_keyframes[-1]
        previous_keyframe_interval = source_adjust_keyframe.time - source_anchor_keyframe.time
        new_keyframe_interval = adjust_target_time - last_updated_keyframe.time

        # To work with map, pass the keyframe ratio information in as nonlocal vars
        def apply_percent_change(animation_keyframe: AnimationKeyframe) -> AnimationKeyframe:
            nonlocal source_anchor_keyframe, previous_keyframe_interval, last_updated_keyframe, new_keyframe_interval
            new_keyframe = copy.deepcopy(animation_keyframe)
            percent_through_interval = (new_keyframe.time - source_anchor_keyframe.time) / previous_keyframe_interval
            new_keyframe.time = percent_through_interval * new_keyframe_interval + last_updated_keyframe.time
            return new_keyframe

        updated_keyframes.extend(
            list(
                map(
                    apply_percent_change,
                    animation_builder.raw_keyframes[anchor_keyframe_idx + 1 : adjust_keyframe_idx + 1],
                )
            )
        )

    # If the final keyframe adjusted wasn't the last keyframe in the sequence
    # move the remaining keyframes so that they maintain the same offsets as before
    if len(updated_keyframes) < animation_builder.keyframe_count:
        change_in_end_timestamp = (
            updated_keyframes[-1].time - animation_builder.raw_keyframes[len(updated_keyframes) - 1].time
        )
        remaining_keyframes_copy = [x for x in animation_builder.raw_keyframes[len(updated_keyframes) :]]
        for keyframe in remaining_keyframes_copy:
            keyframe.time += change_in_end_timestamp
        updated_keyframes.extend(remaining_keyframes_copy)

    # Delete source keyframes and re-fill with adjusted values
    del animation_builder.raw_keyframes[:]
    animation_builder.raw_keyframes.extend(updated_keyframes)


def insert_keyframe_and_adjust_times(
    animation_builder: AnimationBuilder,
    keyframe: AnimationKeyframe,
    after_index: int,
    preceeding_buffer: float,
    trailing_buffer: float,
) -> None:
    """Inserts a keyframe into the animation with explicit buffers on either side"""
    all_keyframes = animation_builder.raw_keyframes
    previous_keyframe_time = 0 if animation_builder.keyframe_count <= after_index else all_keyframes[after_index].time
    keyframe.time = previous_keyframe_time + preceeding_buffer

    # Update the keyframes that come after
    full_offset = preceeding_buffer + trailing_buffer
    for idx in range(after_index + 1, len(all_keyframes)):
        all_keyframes[idx].time = all_keyframes[idx].time + full_offset

    all_keyframes.insert(after_index + 1, keyframe)


def adjust_keyframe_by_time(
    animation_builder: AnimationBuilder, keyframe_idx: int, adjust_by_time: float, adjust_trailing_keyframes: bool
) -> None:
    """
    Adjusts the keyframe by the requested time
    """
    all_keyframes = animation_builder.raw_keyframes
    if keyframe_idx < 0 or keyframe_idx >= len(all_keyframes):
        print(f"ERROR: Keyframe {keyframe_idx} out of range")
        return

    all_keyframes[keyframe_idx].time += adjust_by_time

    if adjust_trailing_keyframes:
        for idx in range(keyframe_idx + 1, len(all_keyframes)):
            all_keyframes[idx].time += adjust_by_time


def perform_keyframe_interpolation(
    start: AnimationKeyframe,
    stop: AnimationKeyframe,
    modified: AnimationKeyframe,
    percent: float,
    logger: Optional[Logger] = None,
) -> bool:
    """Interpolates between two animation keyframes by the requested percent"""
    is_protobuf = hasattr(start, "ListFields")
    if is_protobuf:
        properties = [descriptor.name for descriptor, _ in start.ListFields()]

        for property in properties:
            start_ref = getattr(start, property, None)
            stop_ref = getattr(stop, property, None)
            modified_ref = getattr(modified, property, None)

            result = perform_keyframe_interpolation(start_ref, stop_ref, modified_ref, percent)
            # If interpolation not performed at nested level, use set attribute at this
            # level to update the interpolation
            if not result:
                if isinstance(stop_ref, float) and isinstance(start_ref, float):
                    interpolated_value = (stop_ref - start_ref) * percent + start_ref
                elif isinstance(stop_ref, bool) and isinstance(start_ref, bool):
                    interpolated_value = start_ref
                else:
                    if logger:
                        logger.error(f"Unknown type {type(stop_ref)} for interpolation")
                    raise AssertionError()
                setattr(modified, property, interpolated_value)
    return is_protobuf


def extract_pose_for_animation_time(
    animation: Animation, time_offset_in_animation: float, logger: Optional[Logger] = None
) -> Optional[AnimationKeyframe]:
    """
    Returns the robot's pose at the given offset time from the animation.
    Note that this is not the nearest keyframe to the time, this is a keyframe containing the pose
    at the requested offset. If the time requested is between keyframes, an interpolation will generate the
    pose that the robot would be at in the playback for review
    """
    if animation is None:
        return None

    builder = AnimationBuilder.from_animation(animation)
    builder.get_keyframe_before_timestamp(time_offset_in_animation)

    (prev_keyframe_idx, prev_keyframe) = builder.get_keyframe_before_timestamp(time_offset_in_animation)
    if prev_keyframe_idx + 1 > builder.keyframe_count:
        return None
    next_keyframe = builder.raw_keyframes[prev_keyframe_idx + 1]
    keyframe_time_difference = next_keyframe.time - prev_keyframe.time
    interpolation_percent = (
        (time_offset_in_animation - prev_keyframe.time) / keyframe_time_difference if keyframe_time_difference else 1
    )

    # Start with current keyframe as base so that all protobuf properites are already in place
    output_keyframe = copy.deepcopy(prev_keyframe)
    perform_keyframe_interpolation(prev_keyframe, next_keyframe, output_keyframe, interpolation_percent, logger)
    return output_keyframe


def resample_animation(
    animation_builder: AnimationBuilder, start_time: float, end_time: float, resample_rate: int
) -> None:
    """
    Takes the animation within the animation builder and re-samples all keyframes
    to the resample rate between the start and end time
    """
    base_animation = copy.deepcopy(animation_builder.raw_animation)
    del animation_builder.raw_keyframes[:]

    new_keyframe_count = int((end_time - start_time) * resample_rate)

    new_keyframe_times = np.linspace(start_time, end_time, num=new_keyframe_count)
    for current_time in new_keyframe_times:
        new_keyframe = extract_pose_for_animation_time(
            animation=base_animation,
            time_offset_in_animation=current_time,
            logger=animation_builder._logger,
        )
        if new_keyframe:
            animation_builder.insert_keyframe_at_time(animation_keyframe=new_keyframe, start_time=current_time)


def flatten_keyframe_to_dictionary(keyframe: AnimationKeyframe) -> dict[str, float]:
    """
    Recursively walks through the keyframe to extract all of the set values
    """
    keyframe_to_proto_attrs = joint_angle_to_protobuf_attrs_map()
    flattened_map = {}

    for animation_name, nested_proto_attrs in keyframe_to_proto_attrs.items():
        active_proto = keyframe
        extraction_success = True
        for attribute in nested_proto_attrs:
            if check_if_protobuf_field_set(active_proto, attribute):
                active_proto = getattr(active_proto, attribute)
            else:
                extraction_success = False
        if extraction_success:
            flattened_map[animation_name] = active_proto
        else:
            flattened_map[animation_name] = 0

    return flattened_map
