# Copyright (c) 2024-2025 Boston Dynamics AI Institute LLC. All rights reserved.

import asyncio
import copy
import math
import time
from logging import Logger
from typing import Optional

from bosdyn.api.spot.choreography_sequence_pb2 import (
    Animation,
    AnimationKeyframe,
    ChoreographySequence,
    ChoreographyStatusResponse,
)
from spot_wrapper.wrapper import SpotWrapper

from spot_choreo_utils.choreo_creation.choreo_builders.animation_builder import AnimationBuilder
from spot_choreo_utils.choreo_creation.choreo_builders.animation_operators import (
    create_single_animation_sequence,
    extract_pose_for_animation_time,
)
from spot_choreo_utils.choreo_playback.synced_performance_modality import (
    SyncedPerformanceModality,
    SyncedPeroformanceConfig,
)


class SyncedSpotDancer(SyncedPerformanceModality):
    """
    Lets spot perform in sync with other robots and music by ensuring animation uploads,
    getting to the first keyframe, and other considerations are all handled automatically
    """

    def __init__(
        self,
        username: str,
        password: str,
        hostname: str,
        robot_name: str,
        port: int = 0,
        logger: Optional[Logger] = None,
    ) -> None:
        if logger is None:
            logger = Logger(f"Synced Spot Dancer {robot_name}")

        self._logger = logger
        self._spot_wrapper = SpotWrapper(
            username=username,
            password=password,
            hostname=hostname,
            robot_name=robot_name,
            logger=logger,
            use_take_lease=True,
            port=port,
        )

        self._animation: Optional[Animation] = None
        self._animation_builder: Optional[AnimationBuilder] = None
        self._sequence: Optional[ChoreographySequence] = None
        self._starting_keyframe: Optional[AnimationKeyframe] = None
        self._start_slice = 0

    def set_animation(self, animation_builder: AnimationBuilder) -> bool:
        """
        Set an animation for playback on the robot
        """
        self._animation_builder = animation_builder
        return True

    def set_sequence(self, sequence: ChoreographySequence) -> bool:
        """Set a sequence for playback on robot"""
        self._sequence = sequence
        return True

    def upload_dance_with_start_time(self, start_time_s: float) -> bool:
        """Uploads the animation so that it starts at an arbitrary time elapsed"""
        if self._animation_builder is not None:
            # Create a new animation builder for procedural alterations
            self._animation_builder.set_logger(None)
            local_builder = copy.deepcopy(self._animation_builder)

            # Remove all timestamps before the start time
            all_timestamps = local_builder.keyframe_timestamps
            for timestamp in all_timestamps:
                if timestamp < start_time_s:
                    local_builder.remove_keyframe_at_idx(0, adjust_trailing_keyframes=True)
                else:
                    break

            # Insert an interpolated keyframe for the exact start time if the requested start time
            # does not fall on an exact spot keyframe
            if not math.isclose(0, local_builder.keyframe_timestamps[0]):
                # Insert an interpolation
                tmp_animation = self._animation_builder.build()
                keyframe = extract_pose_for_animation_time(tmp_animation, start_time_s)
                local_builder.insert_keyframe_at_time(keyframe, 0)

            # Extend the animation so the robot holds the final pose
            build_settings = AnimationBuilder.BuildSettings()
            build_settings.hold_final_pose_s = 30

            # Create and upload the new animation, saving references to request playback later
            self._animation, self._sequence = create_single_animation_sequence(local_builder, build_settings)
            self._starting_keyframe = self._animation.animation_keyframes[0]

            anim_res, msg = self._spot_wrapper.upload_animation_proto(self._animation)
            if not anim_res:
                self._logger.error(f"Animation upload failed with error {msg}")
                return False

        choreo_res, msg = self._spot_wrapper.upload_choreography(self._sequence)
        if not choreo_res:
            self._logger.error(f"Choreography upload failed with error {msg}")
        return choreo_res

    def prepare_dance_to_get_to_first_keyframe(self) -> Optional[str]:
        """Creates and uploads an animation that moves the robot to the starting pose for the current dance"""
        if self._starting_keyframe is None:
            return self._sequence.name if self._sequence else None

        starting_keyframe = AnimationBuilder()
        starting_keyframe.start_from_empty("Starting keyframe")

        # This will cause the robot to move to its start pose in 1 second
        # The robot could move quickly if it's far from the start pose
        self._starting_keyframe.time = 0
        starting_keyframe.insert_keyframe_at_index(self._starting_keyframe, 0)
        self._starting_keyframe.time = 1
        starting_keyframe.insert_keyframe_at_index(self._starting_keyframe, 1)

        build_settings = AnimationBuilder.BuildSettings()

        anim, seq = create_single_animation_sequence(starting_keyframe, build_settings)

        anim_res, msg = self._spot_wrapper.upload_animation_proto(anim)
        if not anim_res:
            self._logger.error(f"Animation upload failed with error {anim_res}")
        choreo_res, msg = self._spot_wrapper.upload_choreography(seq)
        if not choreo_res:
            self._logger.error(f"Choreography upload failed with error {choreo_res}")
        return seq.name

    async def prep_performance(self, config: SyncedPeroformanceConfig) -> bool:
        """
        Set up the modality so that it's ready to start instantly when this function returns.
        E.G. A necessary buffer is loaded into memory to start playback etc.
        """
        self._duration = config.end_time_s - config.start_time_s
        self._spot_wrapper.getLease()

        # Upload the dance, edited to start at the appropriate start time
        # TODO: Currently this start time works for single animation sequences
        # needs to be updated for the set_sequence use case
        upload_result = self.upload_dance_with_start_time(config.start_time_s)
        if not upload_result:
            return False

        # If the sequence starts with a BOSDYN move, there's no need to set the robot pose
        # before the dance starts. If it starts with an animation, interpolate to the first
        # keyframe location so that the dance starts at the correct location
        if self._sequence and self._sequence.moves[0].type != "animation":
            return True

        # Now upload and play a dance that is just the starting keyframe
        # This ensures the robot always starts at the appropriate spot for the full dance
        sequence_name = self.prepare_dance_to_get_to_first_keyframe()
        if sequence_name is None:
            return False

        self._spot_wrapper.execute_choreography_by_name(sequence_name, start_slice=0, use_async=False)

        # Wait for the robot to arrive at its starting pose
        start_time = time.time()
        while result := self._spot_wrapper.get_choreography_status():
            if result[2].status == ChoreographyStatusResponse.Status.STATUS_COMPLETED_SEQUENCE:
                return True
            if time.time() - start_time > config.setup_timeout:
                return False
            await asyncio.sleep(0.05)
        return True

    async def start_performance(self) -> None:
        """Start the performance instantly so that all modalities stay in sync"""
        if self._sequence is None:
            self._logger.error("No Sequence, can't perform")
            return

        # Start the dance
        self._spot_wrapper.execute_choreography_by_name(
            self._sequence.name, start_slice=self._start_slice, use_async=True
        )

        # Monitor for the dance to finish of its own accord, or for the configured end time
        start_time = time.time()
        while self._spot_wrapper.get_choreography_status():
            if (time.time() - start_time) > self._duration:
                self._spot_wrapper.stop_choreography()
                break
            else:
                await asyncio.sleep(0.05)

    async def stop_performance(self) -> None:
        """Stop the performance instantly so that all modalities stay in sync"""
        self._spot_wrapper.stop_choreography()
