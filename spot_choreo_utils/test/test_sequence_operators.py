#  Copyright (c) 2023 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.
import copy
import unittest
from pathlib import Path

from spot_choreo_utils.choreo_creation.choreo_builders.sequence_builder import SequenceBuilder

# Functions tested in this file
from spot_choreo_utils.choreo_creation.choreo_builders.sequence_operators import (
    adjust_sequence_by_slices,
    adjust_sequence_by_time,
    remove_zero_length_moves,
)
from spot_choreo_utils.paths import get_example_choreo_path
from spot_choreo_utils.serialization.serialization_utils import load_sequence


class SequenceBuilderUtilsTester(unittest.TestCase):
    def setUp(self) -> None:
        sequence_tango_r1 = Path(get_example_choreo_path(), "sequences", "pose_to_pose_sequence.pbtxt").resolve()
        r1_sequence = load_sequence(sequence_tango_r1.as_posix())
        self._r1_builder = SequenceBuilder()
        self._r1_builder.start_from_sequence(r1_sequence)

    def test_list_move_types(self) -> None:
        """
        Get moves in sequence
        """
        # Known move list for test
        all_moves = self._r1_builder.move_types
        self.assertEqual(all_moves[0], "animation")

    def test_adjust_sequence_by_amount(self) -> None:
        """
        Ensure time and slice adjustments align properly
        """
        sequence_copy_1 = copy.deepcopy(self._r1_builder)
        sequence_copy_2 = copy.deepcopy(self._r1_builder)

        adjustment_time = 0.5
        slices_per_second = self._r1_builder._sequence.slices_per_minute / 60
        adjust_by_slices = int(slices_per_second * adjustment_time)

        adjust_idx = 0

        adjust_sequence_by_time(sequence_copy_1, adjust_idx, adjust_idx + 1, adjustment_time)
        adjust_sequence_by_slices(sequence_copy_2, adjust_idx, adjust_idx + 1, adjust_by_slices)

        self.assertEqual(sequence_copy_1._sequence.moves[adjust_idx], sequence_copy_2._sequence.moves[adjust_idx])
        self.assertNotEqual(sequence_copy_1._sequence.moves[adjust_idx], self._r1_builder._sequence.moves[adjust_idx])

    def test_remove_zero_length_moves(self) -> None:
        """
        Test that invalid moves (length zero or less) are removed
        """
        # Manually set move for removal
        self._r1_builder._sequence.moves[0].requested_slices = 0

        # Remove moves of length 0
        remove_zero_length_moves(self._r1_builder)
        final_length = len(self._r1_builder.move_types)
        self.assertEqual(0, final_length)


if __name__ == "__main__":
    unittest.main()
