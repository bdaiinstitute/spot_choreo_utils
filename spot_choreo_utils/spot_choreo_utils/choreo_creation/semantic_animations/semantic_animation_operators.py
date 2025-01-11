#  Copyright (c) 2024-2025 Boston Dynamics AI Institute LLC. All rights reserved.

import copy

from spot_choreo_utils.choreo_creation.semantic_animations.semantic_animation_builder import SemanticAnimationBuilder


def repeat_named_segment(
    semantic_builder: SemanticAnimationBuilder, segment_name: str, new_segment_name: str, insertion_index: int
) -> None:
    """
    Take a segment of the dance and repeat it at the requested index
    """
    start_stop_pair = semantic_builder.get_segment_range(segment_name)
    if start_stop_pair is None:
        semantic_builder._logger.error(  # noqa: SLF001
            f"Could not repeat segment with name {segment_name}. Not found in semantic animation"
        )

    start_idx = start_stop_pair[0]
    stop_idx = start_stop_pair[1]

    previous_keyframe = semantic_builder.animation_builder.keyframe_at_index(start_idx)
    # Segment names are inclusive, so add one when retrieving keyframes
    repeat_keyframes = copy.deepcopy(semantic_builder.animation_builder.keyframe_range(start_idx, stop_idx + 1))
    # Calculate new relative offsets for all keyframes
    previous_time = previous_keyframe.time if previous_keyframe is not None else 0
    relative_offsets = [keyframe.time - previous_time for keyframe in repeat_keyframes]

    # Calculate new keyframe times using the relative offsets and time at the insertion point
    animation_length = semantic_builder.animation_builder.keyframe_count
    insertion_time_index = insertion_index if insertion_index < animation_length else animation_length - 1
    time_at_insert_point = semantic_builder.animation_builder.keyframe_at_index(insertion_time_index).time

    for update_idx in range(len(repeat_keyframes)):
        repeat_keyframes[update_idx].time = relative_offsets[update_idx] + time_at_insert_point

    # Insert the repeated keyframes into the animation
    semantic_builder.animation_builder.insert_keyframes_at_index(
        repeat_keyframes, insertion_index, adjust_trailing_keyframes=True
    )

    # Update the semantic map with the new segment name.
    # Since end of segments is inclusive, subtract one for keyframe idx.
    end_segment_index = insertion_index + len(repeat_keyframes) - 1
    semantic_builder.name_segment_by_index(new_segment_name, start_index=insertion_index, end_index=end_segment_index)


def reverse_named_segment(semantic_builder: SemanticAnimationBuilder, segment_name: str) -> None:
    """
    Take a segment of the dance and reverse all of the keyframes
    """
    start_stop_pair = semantic_builder.get_segment_range(segment_name)
    if start_stop_pair is None:
        semantic_builder._logger.error(  # noqa: SLF001
            f"Could not repeat segment with name {segment_name}. Not found in semantic animation"
        )
        return

    start_idx = start_stop_pair[0]
    stop_idx = start_stop_pair[1]

    anim_builder = semantic_builder.animation_builder

    while start_idx < stop_idx:
        copy_first_elem = copy.deepcopy(anim_builder.keyframe_at_index(start_idx))
        copy_second_elem = copy.deepcopy(anim_builder.keyframe_at_index(stop_idx))
        copy_first_elem.time = anim_builder.keyframe_at_index(stop_idx).time
        copy_second_elem.time = anim_builder.keyframe_at_index(start_idx).time
        anim_builder.overwrite_keyframe_at_index(copy_first_elem, stop_idx)
        anim_builder.overwrite_keyframe_at_index(copy_second_elem, start_idx)
        start_idx += 1
        stop_idx -= 1

    # Update the segment name to use the reversed segment
    semantic_builder.remove_segment_name(segment_name)
    semantic_builder.name_segment_by_index(segment_name, start_stop_pair[0], start_stop_pair[1])
