#  Copyright (c) 2024-2025 Boston Dynamics AI Institute LLC. All rights reserved.

from pathlib import Path


def get_repo_base_path() -> Path:
    """Absolute path to repo base"""
    return Path(__file__).resolve().parent.parent.parent.resolve()


def get_base_choreo_file_path() -> Path:
    """Path to folder for choreography development"""
    return Path(get_repo_base_path(), "choreo_files")


def get_example_choreo_path() -> Path:
    """Returns the path to the example choreography folder"""
    return Path(get_base_choreo_file_path(), "examples")


def get_active_choreo_path() -> Path:
    """Returns the path to the active choreography folder"""
    return Path(get_base_choreo_file_path(), "active")
