#  Copyright (c) 2023-2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

import asyncio
import time
import unittest
from pathlib import Path

# Functions tested in this file
from spot_choreo_utils.choreo_playback.synced_audio_player import SyncedAudioPlayer
from spot_choreo_utils.choreo_playback.synced_performance_coordinator import (
    SyncedPerformanceCoordinator,
    SyncedPeroformanceConfig,
)
from spot_choreo_utils.paths import get_example_choreo_path


class TestSyncedPlayback(unittest.TestCase):
    """
    Tests basic construction of synced playback classes so that code doesn't merge that won't compile
    or errors immediately. No good way to test actual playback syncronization or connection to robot in the loop
    """

    def setUp(self) -> None:
        music_file = Path(get_example_choreo_path(), "music", "60_bpm.mp3")
        self._audio_player = SyncedAudioPlayer(music_file)
        self._coordinator = SyncedPerformanceCoordinator()
        self._default_config = SyncedPeroformanceConfig()
        self._playback_duration = 0.2
        self._default_config.start_time_s = 1.3
        self._default_config.end_time_s = self._default_config.start_time_s + self._playback_duration

    def test_adding_modalities(self) -> None:
        """
        Add various modalities to the coordinator
        """
        self._coordinator.add_modality(self._audio_player)
        self._coordinator.add_modality(self._audio_player)

        self.assertEqual(len(self._coordinator._modalities), 2)

    def test_single_step_performance(self) -> None:
        """
        Do the full performance with one call
        """
        self._coordinator.add_modality(self._audio_player)
        start_time = time.time()
        asyncio.run(self._coordinator.perform_when_ready(self._default_config))

        time_elapsed = time.time() - start_time
        # Mostly here to make sure that we're not eating up tons of test time
        # so provide a wide enough margin that this isn't inconsistent in CI
        self.assertLessEqual(time_elapsed, self._playback_duration * 8)

        asyncio.run(self._coordinator.stop())


if __name__ == "__main__":
    unittest.main()
