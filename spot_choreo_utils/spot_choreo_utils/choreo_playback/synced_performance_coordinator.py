# Copyright (c) 2024-2025 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

import asyncio

from spot_choreo_utils.choreo_playback.synced_performance_modality import (
    SyncedPerformanceModality,
    SyncedPeroformanceConfig,
)


class SyncedPerformanceCoordinator:
    """Class that handles syncornization between multiple performance modalities"""

    def __init__(self) -> None:
        self._modalities: list[SyncedPerformanceModality] = []

    def add_modality(self, modality: SyncedPerformanceModality) -> None:
        """Add a new performance modality to playback"""
        self._modalities.append(modality)

    def clear(self) -> None:
        """Clears out all coordinator settings so that a new performance can re-use the same instance"""
        self._modalities = []

    async def prep_performance(self, config: SyncedPeroformanceConfig) -> None:
        """Have all of the modalities preprare for the performance"""
        coroutines = [modality.prep_performance(config) for modality in self._modalities]
        await asyncio.gather(*coroutines)

    async def start_performance(self) -> None:
        """Start all the modalities in sync"""
        coroutines = [modality.start_performance() for modality in self._modalities]
        await asyncio.gather(*coroutines)

    async def perform_when_ready(self, config: SyncedPeroformanceConfig, delay_once_ready: float = 0) -> None:
        """Prep and play the performance with a single call"""
        await self.prep_performance(config)
        await asyncio.sleep(delay_once_ready)
        await self.start_performance()

    async def stop(self) -> None:
        """Stop the performance immediately"""
        coroutines = []
        for modality in self._modalities:
            coroutines.append(modality.stop_performance())
        asyncio.gather(*coroutines)
