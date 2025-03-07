# Load Animation
from pathlib import Path

from spot_choreo_utils.choreo_creation.choreo_builders.animation_builder import AnimationBuilder
from spot_choreo_utils.choreo_creation.choreo_builders.animation_operators import (
    create_single_animation_sequence,
)

#
from spot_choreo_utils.choreo_creation.choreo_builders.sequence_builder import SequenceBuilder
from spot_choreo_utils.paths import get_active_choreo_path
from spot_choreo_utils.serialization.serialization_utils import load_animation

# hostname="10.17.30.151"
# robot_name="violet"
# username="user"
# password="bbbdddaaaiii"
# port = 0
# logger = logging.Logger(name="spot_wrapper_logger")
# spot_wrapper = SpotWrapper(
#     username=username,
#     password=password,
#     hostname=hostname,
#     port=port,
#     robot_name=robot_name,
#     logger=logger,
#     use_take_lease=True)
# assert(spot_wrapper.is_valid)
# spot_wrapper.claim()


# ## Testing consecutive animation sequencing/timing
# animation = load_animation(Path(get_active_choreo_path(), "test30fail", "test30fail.pbtxt"))
# animation_builder = AnimationBuilder()
# animation_builder.start_from_animation(animation)
# animation = animation_builder.build()
# animation_proto, seq_proto = create_single_animation_sequence(animation, AnimationBuilder.BuildSettings())
# seq_builder = SequenceBuilder()
# seq_builder.start_from_empty("example", slices_per_minute = 240)
# seq_builder.add_animation(animation, start_time=0.0)
# seq_builder.add_animation(animation, start_time=10.0)
# seq_proto = seq_builder.build()
# # print(f"seq_proto {seq_proto}")
# # res, msg = spot_wrapper.upload_animation_proto(animation_proto)
# # res, msg = spot_wrapper.upload_animation_proto(animation_proto)
# res, msg = spot_wrapper.upload_animation_proto(animation_proto)
# print(f"Animation upload res: {res}, msg: {msg}")
# res, msg = spot_wrapper.upload_choreography(seq_proto)
# print(f"Sequence upload res: {res}, msg: {msg}")
# spot_wrapper.execute_choreography_by_name(seq_proto.name, start_slice=0)


## Testing keyframe field filling-in
animation = load_animation(Path(get_active_choreo_path(), "test30fail", "test30fail.pbtxt"))
animation_builder = AnimationBuilder()
animation_builder.start_from_animation(animation)
animation = animation_builder.build()
animation_proto, seq_proto = create_single_animation_sequence(animation, AnimationBuilder.BuildSettings())
seq_builder = SequenceBuilder()
seq_builder.start_from_empty("example", slices_per_minute=240)
seq_builder.add_animation(animation, start_time=0.0)
seq_proto = seq_builder.build()
# res, msg = spot_wrapper.upload_animation_proto(animation_proto)
# print(f"Animation upload res: {res}, msg: {msg}")
# res, msg = spot_wrapper.upload_choreography(seq_proto)
# print(f"Sequence upload res: {res}, msg: {msg}")
# spot_wrapper.execute_choreography_by_name(seq_proto.name, start_slice=0)

# for idx, keyframe in enumerate(animation_proto.animation_keyframes):
#     print(f"\n idx: {idx}, keyframe: {keyframe}")
#     flattened_keyframe_dictionary = flatten_keyframe_to_dictionary(keyframe)
#     print(f"flattened keyframe dictionary: {flattened_keyframe_dictionary}")


# ## Testing a single animation
# animation = load_animation(Path(get_active_choreo_path(),
# "merritt_animation_03042025", "merritt_animation_03042025.pbtxt"))
# animation_builder = AnimationBuilder()
# animation_builder.start_from_animation(animation)
# animation = animation_builder.build()
# animation_proto, seq_proto = create_single_animation_sequence(animation, AnimationBuilder.BuildSettings())
# print(f"animation_proto {animation_proto}")
# seq_builder = SequenceBuilder()
# seq_builder.start_from_empty("example", slices_per_minute = 240)
# # seq_builder.add_animation(animation, start_time=0.0)
# seq_builder.add_animation(animation, start_time=1.0)
# seq_proto = seq_builder.build()
# print(f"seq_proto {seq_proto}")
# res, msg = spot_wrapper.upload_animation_proto(animation_proto)
# print(f"Animation upload res: {res}, msg: {msg}")
# res, msg = spot_wrapper.upload_choreography(seq_proto)
# print(f"Sequence upload res: {res}, msg: {msg}")
# spot_wrapper.execute_choreography_by_name(seq_proto.name, start_slice=0)

# moves_dict = {
#     # "rotate_body": {
#     #     "start_sec": 0.0,
#     #     "duration_sec": 2.0,
#     #     "roll": 0.05,
#     #     "pitch": 0.05,
#     #     "yaw": 0.0,
#     #     "return_to_start_pose": True
#     # },
#     "sway": {
#         "start_sec": 10.0,
#         "duration_sec": 1.0,
#         "horizontal": 0.161
#     },
#     "sway": {
#         "start_sec": 0.0,
#         "duration_sec": 1.0,
#         "horizontal": 0.162
#     },
#     "sway": {
#         "start_sec": 10.0,
#         "duration_sec": 1.0,
#         "horizontal": 0.164
#     },
#     "sway": {
#         "start_sec": 0.0,
#         "duration_sec": 1.0,
#         "horizontal": 0.163
#     },
#     # "sway": {
#     #     "start_sec": 10.0,
#     #     "duration_sec": 3.0,
#     #     "vertical": 0.080
#     # },
#     "twerk": {
#         "start_sec": 1.0,
#         "duration_sec": 1.0,
#         "height": 0.080
#     },
#     # "twerk": {
#     #     "start_sec": 3.0,
#     #     "duration_sec": 2.0,
#     #     "height": 0.080
#     # }
# }

# moves_list = [
#     {
#         "type": "sway",
#         "start_sec": 1.0,
#         "duration_sec": 0.5,
#         "horizontal": 0.161
#     },
#     {
#         "type": "twerk",
#         "start_sec": 0.0,
#         "duration_sec": 0.5,
#         "height": 0.1
#     },
# ]

# moves_list = [
#     {
#         "type": "rotate_body",
#         "start_sec": 0.0,
#         "duration_sec": 2.0,
#         "roll": 0.1,
#         "pitch": 0.1,
#         "yaw": 0.0,
#         "return_to_start_pose": True
#     },
#     {
#         "type": "twerk",
#         "start_sec": 2.0,
#         "duration_sec": 3.0,
#         "height": 0.05
#     }
# ]

# builder = SequenceBuilder()
# builder.start_from_empty("example", slices_per_minute=516)
# builder.add_moves(moves_list)
# seq_proto = builder.build()
# print(f"seq_proto: {seq_proto}")
# res, msg = spot_wrapper.upload_choreography(seq_proto)
# print(f"choreo upload: {res}, {msg}")
# spot_wrapper.execute_choreography_by_name(seq_proto.name, start_slice=0)
