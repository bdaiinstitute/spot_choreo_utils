# Copyright (c) 2024-2025 Boston Dynamics AI Institute LLC. All rights reserved.

import asyncio
import subprocess
from logging import Logger
from pathlib import Path
from typing import Optional

from spot_choreo_utils.choreo_playback.synced_performance_modality import (
    SyncedPerformanceModality,
    SyncedPeroformanceConfig,
)


class SyncedAudioPlayer(SyncedPerformanceModality):
    def __init__(self, file_path: Path, logger: Optional[Logger] = None) -> None:
        if logger is None:
            logger = Logger("Synced Audio Player")

        self._logger = logger
        self._file_path = file_path
        self._process: Optional[subprocess.Popen] = None

    async def prep_performance(self, config: SyncedPeroformanceConfig) -> None:
        """Capture the config properties - no other setup necessary"""
        self.start_time = config.start_time_s + config.music_offset_s
        self.duration = config.end_time_s - config.start_time_s

    async def start_performance(self) -> None:
        """Start the performance instantly so that all modalities stay in sync"""
        if not self._file_path.exists():
            return
        self._process = subprocess.Popen(
            [
                "ffplay",
                "-ss",
                f"{self.start_time}",
                "-nodisp",
                "-autoexit",
                "-stats",
                "-t",
                str(self.duration),
                "-hide_banner",
                self._file_path,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        while self._process is not None and self._process.poll() is None:
            await asyncio.sleep(0.01)
        self._process = None

    async def stop_performance(self) -> None:
        """Stop the performance instantly so that all modalities stay in sync"""
        if self._process is not None:
            self._process.terminate()
        self._process = None
