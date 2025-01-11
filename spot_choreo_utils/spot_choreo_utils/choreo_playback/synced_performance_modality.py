# Copyright (c) 2024-2025 Boston Dynamics AI Institute LLC. All rights reserved.

import sys
from abc import abstractmethod
from typing import Protocol


class SyncedPeroformanceConfig:
    # Synchronized start time offset across performance modalities
    start_time_s: float = 0
    # Synchronized end time across performance modalities
    end_time_s: float = sys.float_info.max
    # Custom offset for where to start the music track
    music_offset_s: float = 0
    # Maximum time to wait for the modality before failing setup
    setup_timeout: float = 3


class SyncedPerformanceModality(Protocol):
    """Protocol for modalities to perform in sync with each other"""

    @abstractmethod
    async def prep_performance(self, config: SyncedPeroformanceConfig) -> None:
        """
        Set up the modality so that it's ready to start instantly when this function returns.
        E.G. A necessary buffer is loaded into memory to start playback etc.
        """

    @abstractmethod
    async def start_performance(self) -> None:
        """Start the performance instantly so that all modalities stay in sync"""

    @abstractmethod
    async def stop_performance(self) -> None:
        """Stop the performance instantly so that all modalities stay in sync"""
