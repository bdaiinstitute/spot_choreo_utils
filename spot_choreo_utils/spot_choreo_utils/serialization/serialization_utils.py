# Copyright (c) 2023-2025 Boston Dynamics AI Institute LLC. All rights reserved.

import logging
import os
import tempfile
from pathlib import Path
from typing import Optional, Union

from bosdyn.api.spot.choreography_sequence_pb2 import Animation, ChoreographySequence
from bosdyn.choreography.client.animation_file_to_proto import convert_animation_file_to_proto
from bosdyn.choreography.client.choreography import (
    load_choreography_sequence_from_txt_file,
)
from google.protobuf import text_format

from spot_choreo_utils.choreo_creation.choreo_builders.animation_builder import AnimationBuilder
from spot_choreo_utils.choreo_creation.choreo_builders.animation_proto_utils import check_if_protobuf_field_set
from spot_choreo_utils.choreo_creation.choreo_builders.sequence_builder import SequenceBuilder


def load_animation(file_path: Union[str, Path], logger: Optional[logging.Logger] = None) -> Optional[Animation]:
    """Universal loader for choreography animation file formats"""

    if isinstance(file_path, str):
        file_path = Path(file_path).resolve()

    animation_as_proto = None
    if not file_path.exists():
        if logger is not None:
            logger.error(f"No file found at {file_path}")
        return animation_as_proto

    extension = file_path.suffix
    if extension == ".cha":
        # Bosdyn representation
        animation_as_proto = convert_animation_file_to_proto(file_path.as_posix()).proto
    elif extension == ".pbtxt" or extension == ".cap":
        # Protobuf representations
        with open(file_path, "r") as f:
            as_string = f.read()
            reload_animation = text_format.Parse(as_string, Animation())
            animation_as_proto = reload_animation
    else:
        if logger is not None:
            logger.error(f"File type {extension} not recognized. Could not load animation")
        return animation_as_proto

    file_name = file_path.stem
    if file_name != animation_as_proto.name:
        if logger:
            logger.error(
                "File name and animation name don't match. Overwritting animation name"
                f" {animation_as_proto.name} to {file_name}"
            )
        animation_as_proto.name = file_name
    return animation_as_proto


def save_animation(
    animation: Union[Animation, AnimationBuilder],
    output_folder: Union[str, Path],
    logger: Optional[logging.Logger] = None,
) -> Optional[Path]:
    """Save animation to disk as a .pbtxt"""

    if isinstance(output_folder, str):
        output_folder = Path(output_folder).resolve()

    if isinstance(animation, AnimationBuilder):
        animation = animation.build()
        if not animation:
            error_msg = "Failed to build animation, can't save"
            if logger:
                logger.error(error_msg)
            return None

    if not check_if_protobuf_field_set(animation, "name"):
        error_msg = "Animation has no name, can't save"
        if logger:
            logger.error(error_msg)
        return None

    os.makedirs(output_folder, exist_ok=True)

    file_path = Path(output_folder, animation.name).with_suffix(".pbtxt").resolve()
    with open(file_path, "w") as f:
        f.write(f"{animation}")
    return file_path


def load_sequence(file_path: str | Path, logger: Optional[logging.Logger] = None) -> Optional[ChoreographySequence]:
    """Universal loader for choreography sequence file formats"""

    if isinstance(file_path, str):
        file_path = Path(file_path).resolve()

    extension = file_path.suffix
    if extension == ".csq":
        return load_choreography_sequence_from_txt_file(file_path.as_posix())
    elif extension == ".chr":
        # A .chr file is a wrapper around .csq data with metadata for the Choreographer GUI.
        # Boston Dynamics doesn't provide a function to convert a .chr into a .csq
        # .chr and .csq are bespoke formats that aren't quite json, so they have
        # to be edited as strings instead of structured data.
        # By chopping the opening and closing parenthesis off of a .chr
        # it "unwrapps" int a .csq which can then be read in as a .csq
        with open(file_path, "r") as file:
            dance_sequence = file.read()
            # Chop out the first and last parenthesis from the file
            first_paren_idx = dance_sequence.find("{")
            final_paren_idx = dance_sequence.rfind("}")
            dance_sequence = dance_sequence[first_paren_idx + 1 : final_paren_idx]
            # Now write the unwrapped file to temp memory and treat as a .csq
            with tempfile.TemporaryDirectory() as temp_dir:
                filename = "temporary_sequence.csq"
                temp_path = Path(temp_dir, filename)
                with open(temp_path, "w") as sequence_file:
                    sequence_file.write(dance_sequence)
                return load_choreography_sequence_from_txt_file(temp_path)
    elif extension == ".pbtxt":
        with open(file_path, "r") as f:
            as_string = f.read()
            output_seuence = text_format.Parse(as_string, ChoreographySequence())
            return output_seuence
    else:
        if logger:
            logger.error(f"File type {extension} not recognized. Could not load sequence")
    return None


def save_sequence(
    sequence: Union[ChoreographySequence, SequenceBuilder],
    output_folder: Union[str, Path],
    logger: Optional[logging.Logger] = None,
) -> Optional[Path]:
    """Save sequences to disk as a .pbtxt"""

    if isinstance(sequence, SequenceBuilder):
        sequence = sequence.build()
        if not sequence:
            error_msg = "Failed to build sequence, can't save"
            if logger:
                logger.error(error_msg)
            return None

    os.makedirs(output_folder, exist_ok=True)

    file_path = Path(output_folder, f"{sequence.name}.pbtxt")
    with open(file_path, "w") as f:
        f.write(f"{sequence}")
    return file_path
