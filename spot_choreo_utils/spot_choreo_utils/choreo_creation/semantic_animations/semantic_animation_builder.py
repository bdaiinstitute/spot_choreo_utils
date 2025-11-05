#  Copyright (c) 2024-2025 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

import os
from logging import Logger
from pathlib import Path
from typing import Optional

from bosdyn.api.spot.choreography_sequence_pb2 import Animation, AnimationKeyframe
from google.protobuf import text_format

from spot_choreo_utils.choreo_creation.choreo_builders.animation_builder import (
    AnimationBuilder,
    check_if_keyframe_poses_equivalent,
)
from spot_choreo_utils.protos.semantic_animation_pb2 import (
    CustomNamedSegment,
    SemanticAnimation,
    SemanticMetadata,
)
from spot_choreo_utils.serialization.serialization_utils import load_animation


class SemanticAnimationBuilder:
    """Class that provides python helper functions to interact with the semantic animation Proto"""

    def __init__(self, logger: Optional[Logger] = None) -> None:
        self._logger = logger if logger is not None else Logger("Default Semantic Animation")
        self._animation_builder = AnimationBuilder(self._logger)
        self._semantic_metadata = SemanticMetadata()

    @property
    def animation_builder(self) -> AnimationBuilder:
        """
        Returns the internal animation builder
        """
        return self._animation_builder

    def start_from_animation(self, animation_proto: Animation) -> None:
        """
        Create a semantic animation using an existing animation as a basis
        """
        self._animation_builder.start_from_animation(animation_proto)

    def load_animation(self, animation_path: Path) -> None:
        """
        Loads an animation from disk.
        """
        self._animation_builder.start_from_animation(load_animation(animation_path))

    def load_semantic_animation(self, semantic_animation_path: Path) -> None:
        """
        Load a semantic animation pbtxt from disk.
        This file contains both the animation definition that can play back on spot
        and semantic metadata used by this class
        """
        with open(semantic_animation_path, "r") as f:
            file_contents = f.read()
            semantic_animation = text_format.Parse(file_contents, SemanticAnimation())
            self._animation_builder.start_from_animation(semantic_animation.base_animation)
            self._semantic_metadata = semantic_animation.semantic_metadata

    def save_semantic_animation(self, output_folder: Path, name_suffix: str = "") -> Optional[Path]:
        """
        Save the semantic Animation to disk. Returns the file name it was saved to
        """
        is_valid = self.validate()
        if not is_valid:
            self._logger.error("Semantic animation invalid - will not save to disk")
            return None

        os.makedirs(output_folder, exist_ok=True)
        build_settings = AnimationBuilder.BuildSettings()
        build_settings.apply_unique_name = False
        build_settings.hold_final_pose_s = 0
        build_settings.apply_stance_to_all_keyframes = False

        animation_proto = self._animation_builder.build(build_settings)
        file_path = Path(output_folder, f"{animation_proto.name}{name_suffix}.pbtxt").resolve()

        with open(file_path, "w") as f:
            semantic_animation = SemanticAnimation()
            semantic_animation.base_animation.CopyFrom(animation_proto)
            semantic_animation.semantic_metadata.CopyFrom(self._semantic_metadata)
            f.write(f"{semantic_animation}")

        return file_path

    def build(self, build_settings: AnimationBuilder.BuildSettings) -> Optional[Animation]:
        """
        Returns an animation proto that can be played back on spot
        """
        return self._animation_builder.build(build_settings)

    def insert_keyframe_at_time(self, animation_keyframe: AnimationKeyframe, start_time: float) -> None:
        """
        Insert a keyframe into the animation builder and update the semantic metadata as necessary
        """
        self._animation_builder.insert_keyframe_at_time(animation_keyframe, start_time)
        self._standardize_semantic_metadata()

    def remove_keyframe_at_idx(self, keyframe_idx: int, adjust_trailing_keyframes: bool = False) -> None:
        """
        Remove the keyframe and update semantic data
        """
        current_keyframe_at_index = self._animation_builder.keyframe_at_index(keyframe_idx)
        if current_keyframe_at_index is None:
            self._logger.error(
                f"Couldn't remove keyframe at index {keyframe_idx} because animation is of length"
                f" {current_keyframe_at_index}"
            )
            return

        # Check to see if this keyframe is the start or end of a semantic segment
        for named_segment in self._semantic_metadata.named_segments:
            if check_if_keyframe_poses_equivalent(named_segment.start_keyframe, current_keyframe_at_index):
                new_index = min(keyframe_idx + 1, self._animation_builder.keyframe_count - 1)
                named_segment.start_keyframe = self._animation_builder.keyframe_at_index(new_index)
                break
            if check_if_keyframe_poses_equivalent(named_segment.end_keyframe, current_keyframe_at_index):
                new_index = max(0, keyframe_idx - 1)
                named_segment.end_keyframe = self._animation_builder.keyframe_at_index(new_index)
                break

        self._animation_builder.remove_keyframe_at_idx(keyframe_idx, adjust_trailing_keyframes)
        self._standardize_semantic_metadata()

    def name_segment_by_index(self, segment_name: str, start_index: int, end_index: int) -> None:
        """
        Apply a semantic name to a segment of the animation
        """
        animation_length = self._animation_builder.keyframe_count

        if start_index > end_index or end_index >= animation_length or start_index < 0 or end_index < 0:
            insertion_error_message = (
                f"Invalid segment {segment_name} from idx {start_index} to {end_index}. Check for overlap with other"
                f" segments and max keyframe {animation_length - 1}."
            )
            self._logger.error(insertion_error_message)
            return

        # Check that name is unique
        existing_segment_names = [segment.segment_name for segment in self._semantic_metadata.named_segments]
        if segment_name in existing_segment_names:
            self._logger.error(f"Segment named {segment_name} already exist, can't add")
            return

        new_segment = CustomNamedSegment(
            segment_name=segment_name,
            start_keyframe=self._animation_builder.keyframe_at_index(start_index),
            end_keyframe=self._animation_builder.keyframe_at_index(end_index),
        )

        # Locate the current named segment that ends before this and starts after this
        insert_index = len(self._semantic_metadata.named_segments)
        for candidate_idx, named_segment in enumerate(self._semantic_metadata.named_segments):
            current_start_keyframe_idx = self._animation_builder.lookup_index_for_keyframe(named_segment.start_keyframe)
            if end_index < current_start_keyframe_idx:
                insert_index = candidate_idx
                break
        self._semantic_metadata.named_segments.insert(insert_index, new_segment)

        is_valid = self.validate()
        if not is_valid:
            self._logger.error(f"Validation error, can't add segment name {segment_name}")
            del self._semantic_metadata.named_segments[insert_index]

    def get_segment_name_metadata_index(self, segment_name: str) -> Optional[int]:
        """
        Returns the index within the named segment metadata for the segment name
        """
        remove_idx = None
        for candidate_idx, named_segment in enumerate(self._semantic_metadata.named_segments):
            if named_segment.segment_name == segment_name:
                remove_idx = candidate_idx
                break
        return remove_idx

    def remove_segment_name(self, segment_name: str) -> None:
        """
        Removes the name from a segment without modifying the underlying animation
        """
        remove_idx = self.get_segment_name_metadata_index(segment_name)

        if remove_idx is not None:
            del self._semantic_metadata.named_segments[remove_idx]
        else:
            self._logger.error(f"Couldn't remove segment name {segment_name}. Segment not found")

    def delete_animation_segment(self, segment_name: str, adjust_trailing_keyframes: bool = True) -> None:
        """
        Deletes all animation data within a segment name, and optionally adjusts the start time of all
        trailing keyframes by the time of the deleted segment
        """
        start_stop = self.get_segment_range(segment_name)
        if start_stop is None:
            self._logger.error(f"Couldn't find range for segment {segment_name}")
            return

        animation_start_idx, animation_end_idx = start_stop[0], start_stop[1]
        for idx_to_remove in reversed(range(animation_start_idx, animation_end_idx + 1)):
            self._animation_builder.remove_keyframe_at_idx(idx_to_remove, adjust_trailing_keyframes)

        self.remove_segment_name(segment_name)
        self._standardize_semantic_metadata()

    def get_segment_range(self, segment_name: str) -> Optional[tuple[int, int]]:
        """
        Returns the index range for a named segment.
        """
        all_segments = self.get_segment_ranges()
        if segment_name not in all_segments.keys():
            return None
        return all_segments[segment_name]

    def get_segment_ranges(self) -> dict[str, tuple[int, int]]:
        """
        Returns a dictionary that maps all segment names to their start/stop keyframes
        """
        segment_map = dict()
        for named_segment in self._semantic_metadata.named_segments:
            start_keyframe_idx = self._animation_builder.lookup_index_for_keyframe(
                named_segment.start_keyframe, timestamps_should_match=False
            )
            end_keyframe_idx = self._animation_builder.lookup_index_for_keyframe(
                named_segment.end_keyframe, timestamps_should_match=False
            )
            segment_map[named_segment.segment_name] = (start_keyframe_idx, end_keyframe_idx)

        return segment_map

    def validate(self) -> bool:
        """
        Validate the semantic animation meets all requirements for non-overlapping segments,
        animation buildable, etc. Prints any errors and returns false if error encountered.
        """
        builder_result, message = self._animation_builder.validate()
        if not builder_result:
            self._logger.error(message)

        self._standardize_semantic_metadata()

        # Ensure that all start/ends are monotonically increasing
        last_idx_seen = -1
        any_invalid_idxs = False
        for named_segment in self._semantic_metadata.named_segments:
            start_keyframe_idx = self._animation_builder.lookup_index_for_keyframe(named_segment.start_keyframe)
            end_keyframe_idx = self._animation_builder.lookup_index_for_keyframe(named_segment.end_keyframe)

            if start_keyframe_idx <= last_idx_seen:
                self._logger.error(
                    f"Segment {named_segment.segment_name} has invalid start keyframe with index"
                    f" {start_keyframe_idx} but last end keyframe was {last_idx_seen}"
                )
                any_invalid_idxs = True
            last_idx_seen = start_keyframe_idx

            if end_keyframe_idx < last_idx_seen:
                self._logger.error(
                    f"Segment {named_segment.segment_name} has invalid end keyframe with index {end_keyframe_idx} but"
                    f" the start keyframe was {last_idx_seen}"
                )
                any_invalid_idxs = True
            last_idx_seen = end_keyframe_idx

        return builder_result and not any_invalid_idxs

    def _standardize_semantic_metadata(self) -> None:
        """
        Ensures that the semantic metadata keeps a standardized format i.e. no keyframes can be none
        """
        if self._animation_builder.keyframe_count == 0:
            del self._semantic_metadata.named_segments[:]
            return

        segments_to_remove = []
        # Make sure keyframe timestamps are up to date - they can change due to insertions/deletions
        for segment_idx, named_segment in enumerate(self._semantic_metadata.named_segments):
            start_keyframe_idx = self._animation_builder.lookup_index_for_keyframe(
                named_segment.start_keyframe, timestamps_should_match=False
            )
            end_keyframe_idx = self._animation_builder.lookup_index_for_keyframe(
                named_segment.end_keyframe, timestamps_should_match=False
            )

            if start_keyframe_idx < 0 or end_keyframe_idx < 0:
                self._logger.error(
                    f"{named_segment.segment_name} keyframes couldn't be found. Were likely deleted, removing section"
                )
                segments_to_remove.append(segment_idx)
                continue

            named_segment.start_keyframe.CopyFrom(self._animation_builder.keyframe_at_index(start_keyframe_idx))
            named_segment.end_keyframe.CopyFrom(self._animation_builder.keyframe_at_index(end_keyframe_idx))

        for segment_idx in reversed(segments_to_remove):
            del self._semantic_metadata.named_segments[segment_idx]
