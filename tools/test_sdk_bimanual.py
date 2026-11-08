"""Regression checks against the installed SDK; no hardware or inference."""
from types import SimpleNamespace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
import contextlib
import io
import numpy as np
from robots import bodies, control, types
from dream.pacing import Pacer
from sdk_bimanual import HoldLeftClient, RequestBudget, parse_args, open_single_client, RIGHT_MULTITASK, RIGHT_UNPLUG


def action(left, right):
    return types.Action(types.ArmCommand(np.zeros(6), left), types.ArmCommand(np.zeros(6), right))


class SDKTimingTests(unittest.TestCase):
    def make_loop(self):
        pacer = Pacer(action_hz=20, sync_above_ms=900)
        client = SimpleNamespace(refill_due=pacer.refill_due, mode="sync", session_id=None)
        return control.ControlLoop({s: SimpleNamespace(dof=6) for s in ("left", "right")}, [], client,
                                   task="test", control_hz=20, action_horizon=48, action_timeout_s=3,
                                   max_ee_speed_m_s=0.08, max_gripper_step=0.05,
                                   command_thread=False, wire=bodies.METAL.wire())

    def test_both_grippers_are_limited_on_chunk_boundary(self):
        loop = self.make_loop()
        loop._last_action = action(0.05, 0.60849)
        result = loop._clamp(action(1.0, 0.06)).action
        self.assertAlmostEqual(result.left.gripper, 0.10)
        self.assertAlmostEqual(result.right.gripper, 0.55849)

    def test_hold_keeps_all_rows_despite_elapsed_roundtrip(self):
        loop = self.make_loop()
        loop._rows_since_observation = 0
        chunk = types.ActionChunk([action(0.5, 0.5) for _ in range(48)], observed_at_s=10)
        loop._install_chunk(chunk, 12, 2)
        self.assertEqual(len(loop._buffer), 48)

    def test_moving_skips_only_rows_actually_executed(self):
        loop = self.make_loop()
        loop._rows_since_observation = 7
        chunk = types.ActionChunk([action(0.5, 0.5) for _ in range(48)], observed_at_s=10)
        loop._install_chunk(chunk, 12, 2)
        self.assertEqual(len(loop._buffer), 41)

    def test_slow_connection_runs_complete_chunk_before_refill(self):
        p = Pacer(action_hz=20, sync_above_ms=900)
        p.record(1.2)
        self.assertFalse(p.refill_due(30, 18))
        self.assertTrue(p.refill_due(48, 0))

    def test_budget_blocks_third_sdk_request_after_buffer_drains(self):
        with tempfile.TemporaryDirectory() as directory:
            client = Mock()
            client.refill_due.return_value = True
            client.get_action.return_value = {"test": "reply"}
            budget = RequestBudget(client, 2, Path(directory))
            loop = self.make_loop()
            loop._client = budget
            anchor = {"left_arm": [0] * 6, "right_arm": [0] * 6}
            for _ in range(2):
                self.assertTrue(loop._refill_due())
                budget.get_action(commanded=anchor)
            self.assertFalse(loop._refill_due())
            with self.assertRaisesRegex(RuntimeError, "budget exhausted"):
                budget.get_action(commanded=anchor)
            self.assertEqual(client.get_action.call_count, 2)
            self.assertEqual(budget.succeeded, 2)
            client.get_action.assert_called_with(commanded=anchor)

    def test_failed_requests_also_consume_budget(self):
        with tempfile.TemporaryDirectory() as directory:
            client = Mock()
            client.get_action.side_effect = TimeoutError("test timeout")
            budget = RequestBudget(client, 2, Path(directory))
            for _ in range(2):
                with self.assertRaises(TimeoutError):
                    budget.get_action()
            self.assertFalse(budget.refill_due(48, 0))
            self.assertEqual(budget.calls, 2)
            self.assertEqual(budget.succeeded, 0)

    def test_right_only_replaces_all_left_rows_and_preserves_right_targets(self):
        right = np.arange(48 * 7, dtype=float).reshape(48, 7)
        predicted_left = np.ones((48, 7))
        source = {"q_l_chunk": predicted_left, "q_r_chunk": right}
        client = Mock()
        client.get_action.return_value = source
        pose = np.array([0.1, -0.2, 0.3, 0.1, -0.1, 0.2])
        held = HoldLeftClient(client, pose, 0.4, 48)
        pose[:] = 9  # Captured hold pose must not change with later snapshots.
        anchor = {"q_r_chunk": np.zeros(6)}
        result = held.get_action(commanded=anchor)
        np.testing.assert_array_equal(result["q_r_chunk"], right)
        np.testing.assert_array_equal(result["q_l_chunk"],
            np.tile([0.1, -0.2, 0.3, 0.1, -0.1, 0.2, 0.4], (48, 1)))
        np.testing.assert_array_equal(source["q_l_chunk"], predicted_left)
        client.get_action.assert_called_once_with(commanded=anchor)


class FineTunedEntryTests(unittest.TestCase):
    def test_stack_preset_selects_multitask_finetune_and_holds_left(self):
        args = parse_args(["--stack-three", "--training-archive", "matching.zip", "--output", "unused"])
        self.assertEqual(args.right_checkpoint, RIGHT_MULTITASK)
        self.assertTrue(args.right_only)
        self.assertFalse(args.live)

    def test_multitask_requires_matching_archive_instead_of_unplug_default(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["--stack-three", "--output", "unused"])

    def test_unplug_checkpoint_cannot_be_used_for_stacking(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["--stack-three", "--right-checkpoint", RIGHT_UNPLUG, "--output", "unused"])

    def test_single_arm_finetune_uses_adapter_and_never_opens_base_client(self):
        args = parse_args(["--right-only", "--right-checkpoint", RIGHT_UNPLUG, "--output", "unused"])
        with patch("fine_tuned_client.RightFineTuneClient.open") as fine, patch("robots.run.open_client") as base:
            self.assertIs(open_single_client(args, {}), fine.return_value)
            fine.assert_called_once_with(RIGHT_UNPLUG, args.training_archive, args.output)
            base.assert_not_called()

    def test_finetune_failure_propagates_without_base_fallback(self):
        args = parse_args(["--right-only", "--right-checkpoint", RIGHT_UNPLUG, "--output", "unused"])
        with patch("fine_tuned_client.RightFineTuneClient.open", side_effect=ValueError("metadata mismatch")), \
                patch("robots.run.open_client") as base:
            with self.assertRaisesRegex(ValueError, "metadata mismatch"):
                open_single_client(args, {})
            base.assert_not_called()


if __name__ == "__main__":
    unittest.main()
