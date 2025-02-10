# Copyright (c) 2023-2025 Boston Dynamics AI Institute LLC. All rights reserved.
#
# Script that builds pb2 from .proto definitions.
# This script creates a build directory structure that mirrors package needs
# for python imports, copies dependencies from bosdyn, and outputs combined pb2s

import glob
import os
import shutil
import subprocess
from pathlib import Path

from spot_choreo_utils.paths import get_repo_base_path

SOURCE_BOSDYN_PROTOS = Path(get_repo_base_path(), "external", "spot-sdk", "protos", "bosdyn")


def spot_choreo_proto_dir() -> Path:
    """Path to directory with spot_choreo_utils protobufs"""
    return Path(Path(__file__).parent, "src")


def setup_bosdyn_protos(tool_working_directory: Path) -> None:
    """Copy the bosdyn protos into the directory structure required for the protobuf build tools"""
    LOCAL_BUILD_PATH = Path(tool_working_directory, "bosdyn")
    if not LOCAL_BUILD_PATH.exists():
        shutil.copytree(SOURCE_BOSDYN_PROTOS, LOCAL_BUILD_PATH)


def setup_proto_build_directory(tool_working_directory: Path, build_directory: Path, proto_package_name: str) -> None:
    """
    grpc_tools.protoc uses directory structure to inform protobuf package names
    this function creates a build directory that mirrors the path from python package base
    to the export location so that grpc_tools can run from this directory with the proper
    final package structure
    """
    ai_institute_source = spot_choreo_proto_dir()

    # For protobufs and python/packages to play nicely, the build
    # directory structure must match the package naming stucture
    proto_package_output = Path(build_directory, proto_package_name)
    os.makedirs(proto_package_output, exist_ok=True)

    # Copy the source files into the build directory
    institute_protos = glob.glob(Path(ai_institute_source, "*.proto").as_posix())
    for proto in institute_protos:
        os.system(f"cp {proto} {proto_package_output}")


def build_grpc(build_directory: Path, proto_package_name: str) -> str:
    """
    Generate pb2 and grpc files for all AI Institute protobufs.
    The protoc generator uses these rules for generation:
      -I: This is the input path base. All recursive searches for imports and the relative
          package paths will start from this directory. For a pb2 to receive
          the python package name folder_1.folder_2 its source needs to be stored in that
          directory, and all imports must be relative to -I
      -python_out/grpc_out: Folder to write the generated pb2 and grpc files
      -final argument: Path to the .proto for generation. Must be contained within
        the -I directory
    """

    ai_institute_source = spot_choreo_proto_dir()
    institute_protos = glob.glob(Path(ai_institute_source, "*.proto").as_posix())
    # Turn each protofile into both a pb2 and grpc file
    for proto_file in institute_protos:
        output_path = Path(proto_package_name, os.path.basename(proto_file))
        proto_build_command = [
            "python",
            "-m",
            "grpc_tools.protoc",
            "-I",
            f"{build_directory.as_posix()}",
            f"--python_out={build_directory.as_posix()}",
            output_path.as_posix(),
        ]
        subprocess.check_call(proto_build_command)
    return proto_package_name


def copy_into_source_control(build_directory: Path, source_control_dir: Path, proto_package_name: str) -> None:
    """
    Copy outputs out of build directory into source control so that this script is only run
    when changes are made to the proto
    """
    path_to_generated_protos = Path(build_directory, proto_package_name)
    all_pb2_outputs = glob.glob(Path(path_to_generated_protos, "*.py").as_posix())

    for pb2_file in all_pb2_outputs:
        shutil.copy(pb2_file, source_control_dir)
        source_controlled_path = Path(source_control_dir, os.path.basename(pb2_file))
        # Edit the local saved file so that it conforms to line length limits
        conform_output_to_lint_standards(source_controlled_path)


def conform_output_to_lint_standards(file_path: Path) -> None:
    """
    grpc_tools outputs files with arbitrary (extremely long) line lengths. Edit
    these lines so that the outputs can be tracked in version control
    """

    as_string = ""
    serializable_header = "AddSerializedFile(b'"
    with open(file_path, "r") as file_handle:
        while line := file_handle.readline():
            # Find the line that runs really long as a serialized dexriptor
            if serializable_header in line:
                starting_idx = line.find(serializable_header)
                starting_idx += len(serializable_header)
                # Open the parenthesis on the first line
                as_string += line[:starting_idx]
                max_ending_index = starting_idx
                # For remainder of description, split the line at a safe character
                # and append new start of line/end line escapes to preserve multiline
                # concatination
                while max_ending_index < len(line) - 1:
                    # Finish the line with a \ and then start a new line
                    as_string += "'\\\n"
                    # Identify the maximum line length for the next segment
                    # Some special characters, like \x, are tricky to split across
                    # lines, so just find an ending index that avoids this issue
                    max_ending_index = starting_idx + 110
                    if max_ending_index > len(line):
                        max_ending_index = len(line)
                    valid_line_splits = [chr(x) for x in range(ord("g"), ord("x"))] + [")"]
                    while line[max_ending_index - 1] not in valid_line_splits:
                        max_ending_index -= 1

                    # Start the new line with a binary string specifier, then append next segment
                    as_string += "b'" + line[starting_idx:max_ending_index]
                    starting_idx = max_ending_index
            elif len(line) > 120:
                as_string += ",\n".join(line.split(","))
            # Meets line length requirement, just add as is
            else:
                as_string += line

    with open(file_path, "w") as file_handle:
        file_handle.write(as_string)


def clean_build_files(tool_working_directory: Path, build_directory: Path) -> None:
    """Remove all files from previous builds"""
    local_bosdyn_source = Path(tool_working_directory, "bosdyn")
    if local_bosdyn_source.exists():
        shutil.rmtree(Path(tool_working_directory, "bosdyn"))
    if build_directory.exists():
        shutil.rmtree(build_directory)


def build_protos(tool_working_directory: Path, build_directory: Path, proto_package_name: str) -> None:
    """
    Build AI Institute Protos. Since these depend on boston dynamics protos which aren't
    part of the institute's native directory structure, a custom build tree is set up that
    mirrors all python package names for protoc generation.
    """
    setup_bosdyn_protos(build_directory)
    setup_proto_build_directory(tool_working_directory, build_directory, proto_package_name)
    build_grpc(build_directory, proto_package_name)

    # Save local copies of python outputs
    source_control_dir = Path(__file__).parent
    copy_into_source_control(build_directory, source_control_dir, proto_package_name)


if __name__ == "__main__":
    tool_working_directory = Path(__file__).parent
    build_directory = Path(tool_working_directory, "build")
    proto_package_name = "spot_choreo_utils/protos"

    clean_build_files(tool_working_directory, build_directory)
    build_protos(tool_working_directory, build_directory, proto_package_name)
