from spot_choreo_utils.choreo_playback.synced_audio_player import SyncedAudioPlayer
from spot_choreo_utils.choreo_playback.synced_performance_coordinator import SyncedPerformanceCoordinator, SyncedPeroformanceConfig
from spot_choreo_utils.choreo_playback.synced_spot_dancer import SyncedSpotDancer



import logging

hostname="10.17.30.151"
robot_name="violet"
username="user"
password="bbbdddaaaiii"
has_arm = True
port = 0
logger = logging.Logger(name="spot_wrapper_logger")


spot_one = SyncedSpotDancer(username=username, password=password, hostname=hostname, 
                               robot_name=robot_name, logger=logger, port=port)


from spot_choreo_utils.choreo_creation.choreo_builders.animation_builder import (
    AnimationBuilder,
)
from spot_choreo_utils.serialization.serialization_utils import load_animation, load_sequence, save_animation
from spot_choreo_utils.paths import get_example_choreo_path
from pathlib import Path


# Play Animation Example
dance_path = Path(get_example_choreo_path(), "animations", "pose_to_pose_animation.pbtxt")
animation = load_animation(dance_path)
builder = AnimationBuilder.from_animation(animation)
spot_one.set_animation(builder)

# Play Sequence Example
#sequence_path = Path(get_example_choreo_path(), "sequences", "pose_to_pose_sequence.pbtxt")
#sequence = load_sequence(sequence_path)
#spot_one.set_sequence(sequence)


from pathlib import Path
from spot_choreo_utils.paths import get_example_choreo_path

audio_file = Path(get_example_choreo_path(), "music", "60_bpm.wav")
print(audio_file)
audio_player = SyncedAudioPlayer(audio_file)

coordinator = SyncedPerformanceCoordinator()
coordinator.add_modality(spot_one)
coordinator.add_modality(audio_player)


config = SyncedPeroformanceConfig()
config.start_time_s = 0
config.end_time_s = 100
config.music_offset_s = 4
config.setup_timeout = 3
delay_once_ready = 2


await coordinator.perform_when_ready(config, delay_once_ready=delay_once_ready)
