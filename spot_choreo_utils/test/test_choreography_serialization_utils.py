#  Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.
import tempfile
import unittest
from pathlib import Path

from bosdyn.api.spot.choreography_sequence_pb2 import Animation as AnimationProto
from bosdyn.api.spot.choreography_sequence_pb2 import ChoreographySequence as ChoreographySequenceProto

import spot_choreo_utils.serialization.serialization_utils as serialization_utils
from spot_choreo_utils.paths import get_example_choreo_path


class ChreographySerializationTester(unittest.TestCase):
    def setUp(self) -> None:
        self.test_csq_path = Path(get_example_choreo_path(), "sequences", "pose_to_pose_sequence.pbtxt")
        self.test_cha_path = Path(get_example_choreo_path(), "animations", "pose_to_pose_animation.pbtxt")

    def test_load_animations(self) -> None:
        """Load an animation file from disk"""
        # Load using a string
        animation_proto = serialization_utils.load_animation(self.test_cha_path)
        self.assertIsNotNone(animation_proto)
        self.assertIsInstance(animation_proto, AnimationProto)

        # Load using a path
        animation_proto = serialization_utils.load_animation(Path(self.test_cha_path).resolve())
        self.assertIsNotNone(animation_proto)
        self.assertIsInstance(animation_proto, AnimationProto)

    def save_animation(self) -> None:
        """Save an animation as a pbtxt so it can be read back in later"""
        original_proto = serialization_utils.load_animation(self.test_cha_path)
        with tempfile.TemporaryDirectory() as temp_dir:
            # Save using a string
            output_path = serialization_utils.save_animation(original_proto, temp_dir)
            reloaded_animation = serialization_utils.load_animation(output_path)
            self.assertEqual(original_proto, reloaded_animation)

            # Save using a path
            output_path = serialization_utils.save_animation(original_proto, Path(temp_dir).resolve())
            reloaded_animation = serialization_utils.load_animation(output_path)
            self.assertEqual(original_proto, reloaded_animation)

    def test_load_sequence(self) -> None:
        """Load a sqeuence file from disk"""
        # Load using string
        sequence_proto = serialization_utils.load_sequence(self.test_csq_path)
        self.assertIsNotNone(sequence_proto)
        self.assertIsInstance(sequence_proto, ChoreographySequenceProto)

        # Load using path
        sequence_proto = serialization_utils.load_sequence(Path(self.test_csq_path).resolve())
        self.assertIsNotNone(sequence_proto)
        self.assertIsInstance(sequence_proto, ChoreographySequenceProto)

    def test_save_sequence(self) -> None:
        """Save a sequence so that it can be read back later"""
        original_sequence = serialization_utils.load_sequence(self.test_csq_path)
        with tempfile.TemporaryDirectory() as temp_dir:
            # Save using string
            output_path = serialization_utils.save_sequence(original_sequence, temp_dir)
            reloaded_sequence = serialization_utils.load_sequence(output_path)
            self.assertEqual(original_sequence, reloaded_sequence)

            # Save using path
            output_path = serialization_utils.save_sequence(original_sequence, Path(temp_dir).resolve())
            reloaded_sequence = serialization_utils.load_sequence(output_path)
            self.assertEqual(original_sequence, reloaded_sequence)


if __name__ == "__main__":
    unittest.main()
