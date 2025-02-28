# Load Animation
from pathlib import Path

from spot_choreo_utils.choreo_creation.choreo_builders.animation_builder import AnimationBuilder
from spot_choreo_utils.choreo_creation.choreo_builders.animation_operators import create_single_animation_sequence
from spot_choreo_utils.paths import get_active_choreo_path
from spot_choreo_utils.serialization.serialization_utils import load_animation

# spot_wrapper.upload_animation_proto(animation)

# # Save the new animation to disk
# build_settings = AnimationBuilder.BuildSettings()
# build_settings.apply_unique_name = False

# animation_proto = animation_builder.build(build_settings)
# save_animation(animation_proto, "new_animation")

# # Load the example animation
# example_anim = load_animation("new_animation/new_animation.pbtxt")

# animation_builder = AnimationBuilder()
# animation_builder.start_from_animation(example_anim)

build_settings = AnimationBuilder.BuildSettings()
build_settings.hold_final_pose_s = 3

animation = load_animation(Path(get_active_choreo_path(), "test24", "test24.pbtxt"))


## Need to temporarily convert to AnimationBuilder in order to build the animation created with web animator
animation_builder = AnimationBuilder()
animation_builder.start_from_animation(animation)
animation = animation_builder.build()

animation_proto, seq_proto = create_single_animation_sequence(animation, build_settings)

# Upload to robot
# res, msg = spot_wrapper.upload_animation_proto(animation_proto)
# print("Upload animation proto: " + str(res) + "   " + str(msg))
# res, msg = spot_wrapper.upload_choreography(seq_proto)
# print("Upload choreo proto: " + str(res) + "   " + str(msg))
