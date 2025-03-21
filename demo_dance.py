import logging
from pathlib import Path

from spot_choreo_utils.choreo_creation.choreo_builders.sequence_builder import SequenceBuilder
from spot_choreo_utils.choreo_playback.synced_audio_player import SyncedAudioPlayer
from spot_choreo_utils.choreo_playback.synced_performance_coordinator import (
    SyncedPerformanceCoordinator,
    SyncedPeroformanceConfig,
)
from spot_choreo_utils.choreo_playback.synced_spot_dancer import SyncedSpotDancer
from spot_choreo_utils.paths import get_example_choreo_path

# hostname="10.17.30.34"
# robot_name="opal"
hostname = "10.17.30.151"
robot_name = "violet"
username = "user"
password = "bbbdddaaaiii"
has_arm = True
port = 0
logger = logging.Logger(name="spot_wrapper_logger")


spot_one = SyncedSpotDancer(
    username=username, password=password, hostname=hostname, robot_name=robot_name, logger=logger, port=port
)


logger = logging.Logger("demo dance")
seq_bldr = SequenceBuilder(logger)
seq_bldr.start_from_empty("spot_inferno", slices_per_minute=516)

moves_list = [
    #                                     {
    #     "type": "butt_circle",
    #     "requested_slices": 8,
    #     "radius": 0.08,
    #     "number_of_circles": 2,
    # },
    #     {
    #     "type": "fidget_stand",
    #     "start_slice": 0,
    #     "preset": 6,
    #     "requested_slices": 34,
    # },
    #            {
    #     "type": "unstow",
    #     "start_slice": 36,
    #     "requested_slices": 12,
    # },
    #            {
    #     "type": "workspace_arm_move",
    #     "requested_slices": 72,
    #     "absolute": False,
    #     "translation_z": 0.06
    # },
    #                {
    #     "type": "stow",
    #     "requested_slices": 8,
    # },
    #            {
    #     "type": "sway",
    #     "start_slice": 0,
    #     "requested_slices": 42,
    #     "vertical": 0.1,
    #     "horizontal": 0.10,
    #     "roll": 0.10
    # },
    {
        "type": "sway",
        # "start_slice": 42,
        "requested_slices": 4,
        "vertical": 0.07,
        "pivot": 3,
        "horizontal": 0.00,
    },
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "pivot": 3, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "pivot": 3, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "pivot": 3, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "pivot": 3, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": 0.08},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": -0.08},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "roll": -0.15, "vertical": 0.05, "horizontal": -0.05},
    {"type": "sway", "requested_slices": 4, "roll": 0.15, "vertical": 0.05, "horizontal": 0.05},
    {"type": "sway", "requested_slices": 4, "vertical": 0.09, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.09, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.09, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.09, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.09, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.09, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "horizontal": -0.15, "vertical": 0.05},
    {"type": "sway", "requested_slices": 4, "horizontal": 0.15, "vertical": 0.05},
    {"type": "sway", "requested_slices": 4, "vertical": 0.09, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.09, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.10, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.10, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.11, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.11, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.05, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.05, "horizontal": 0.00},
    {
        "type": "unstow",
        "start_slice": 108,
        "requested_slices": 8,
    },
    {
        "type": "workspace_arm_move",
        "requested_slices": 14,
        "absolute": False,
        # "translation_x": -0.8,
        "translation_z": 0.12,
    },
    {
        "type": "chicken_head",
        "requested_slices": 144,
        "bob_magnitude_x": 0.02,
        "bob_magnitude_z": 0.01,
        "bob_magnitude_y": 0.02,
        "beats_per_cycle": 1,
    },
    {
        "type": "sway",
        "start_slice": 120,
        "requested_slices": 4,
        "pivot": 1,
        "vertical": 0.05,
    },
    {
        "type": "sway",
        "requested_slices": 4,
        "pivot": 1,
        "vertical": 0.05,
    },
    {
        "type": "sway",
        "requested_slices": 4,
        "pivot": 1,
        "horizontal": -0.20,
    },
    {
        "type": "sway",
        "requested_slices": 4,
        "pivot": 1,
        "horizontal": -0.20,
    },
    {
        "type": "sway",
        "requested_slices": 4,
        "pivot": 1,
        "horizontal": 0.20,
    },
    {
        "type": "sway",
        "requested_slices": 4,
        "pivot": 1,
        "horizontal": 0.20,
    },
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": 0.00},
    {
        "type": "sway",
        "requested_slices": 4,
        "pivot": 1,
        "horizontal": 0.20,
    },
    {
        "type": "sway",
        "requested_slices": 4,
        "pivot": 1,
        "horizontal": 0.20,
    },
    {
        "type": "sway",
        "requested_slices": 4,
        "pivot": 1,
        "horizontal": -0.20,
    },
    {
        "type": "sway",
        "requested_slices": 4,
        "pivot": 1,
        "horizontal": -0.20,
    },
    #                        {
    #     "type": "chicken_head",
    #     "requested_slices": 16,
    #     "bob_magnitude_x": 0.02,
    #     "bob_magnitude_z": 0.01,
    #     "bob_magnitude_y": 0.02,
    #     "beats_per_cycle": 1
    # },
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.07, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "pivot": 1, "vertical": 0.10, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "pivot": 1, "vertical": 0.14, "horizontal": 0.00},
    {
        "type": "sway",
        "requested_slices": 4,
        "pivot": 1,
        "horizontal": -0.20,
    },
    {
        "type": "sway",
        "requested_slices": 4,
        "pivot": 1,
        "horizontal": -0.20,
    },
    {
        "type": "sway",
        "requested_slices": 4,
        "pivot": 1,
        "horizontal": 0.20,
    },
    {
        "type": "sway",
        "requested_slices": 4,
        "pivot": 1,
        "horizontal": 0.20,
    },
    {"type": "sway", "requested_slices": 4, "vertical": 0.10, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "vertical": 0.10, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "pivot": 1, "vertical": 0.10, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "pivot": 1, "vertical": 0.10, "horizontal": 0.00},
    {"type": "sway", "requested_slices": 4, "pivot": 1, "vertical": 0.14, "horizontal": -0.12},
    {"type": "sway", "requested_slices": 4, "pivot": 1, "vertical": 0.14, "horizontal": 0.12},
    {"type": "sway", "requested_slices": 4, "pivot": 1, "vertical": 0.14, "horizontal": -0.12},
    {"type": "sway", "requested_slices": 4, "pivot": 1, "vertical": 0.14, "horizontal": 0.12},
    {"type": "sway", "requested_slices": 4, "pivot": 3, "vertical": 0.14, "horizontal": -0.14},
    {"type": "sway", "requested_slices": 4, "pivot": 3, "vertical": 0.14, "horizontal": 0.14},
    {"type": "sway", "requested_slices": 4, "pivot": 2, "vertical": 0.17, "horizontal": -0.14},
    {"type": "sway", "requested_slices": 4, "pivot": 2, "vertical": 0.20, "horizontal": 0.14},
    #                               {
    #     "type": "rotate_body_sharp",
    #     "start_slice": 128,
    #     "requested_slices": 4,
    #     # "rotation_roll": 0.06,
    #     "pitch": 0.06,
    #     # "rotation_yaw": 0.06,
    #     "return_to_start_pose": True
    # },
    #                               {
    #     "type": "rotate_body_sharp",
    #     "requested_slices": 4,
    #     # "rotation_roll": 0.06,
    #     "pitch": 0.06,
    #     # "rotation_yaw": 0.06,
    #     "return_to_start_pose": True
    # },
    #                                   {
    #     "type": "rotate_body_sharp",
    #     "requested_slices": 4,
    #     # "rotation_roll": 0.06,
    #     "pitch": 0.06,
    #     # "rotation_yaw": 0.06,
    #     "return_to_start_pose": True
    # },
    #                                   {
    #     "type": "rotate_body_sharp",
    #     "requested_slices": 4,
    #     # "rotation_roll": 0.06,
    #     "pitch": 0.06,
    #     # "rotation_yaw": 0.06,
    #     "return_to_start_pose": True
    # },
    #                       {
    #     "type": "twerk",
    #     "start_slice": 128,
    #     "requested_slices": 4,
    #     "height": 0.06,
    # },
    #                           {
    #     "type": "twerk",
    #     "requested_slices": 4,
    #     "height": 0.06,
    # },
    #                           {
    #     "type": "twerk",
    #     "requested_slices": 4,
    #     "height": 0.06,
    # },
    #                           {
    #     "type": "twerk",
    #     "requested_slices": 4,
    #     "height": 0.06,
    # },
]
seq_bldr.add_moves(moves_list)

print(f"seq_bldr._sequence.moves: {seq_bldr._sequence.moves}")

seq = seq_bldr.build()
print(f"seq: {seq}")


spot_one.set_sequence(seq)


audio_file = Path(get_example_choreo_path(), "music", "60_bpm.wav")
print(audio_file)
audio_player = SyncedAudioPlayer(audio_file)

coordinator = SyncedPerformanceCoordinator()
coordinator.add_modality(spot_one)
coordinator.add_modality(audio_player)


config = SyncedPeroformanceConfig()
config.start_time_s = 0

config.end_time_s = 100
config.music_offset_s = 19.5
config.setup_timeout = 0
delay_once_ready = 2


await coordinator.perform_when_ready(config, delay_once_ready=delay_once_ready)
# coordinator.perform_when_ready(config, delay_once_ready=delay_once_ready)
