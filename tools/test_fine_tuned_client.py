"""Verify training-order mapping, percent grippers, and mixed-model time coordinates."""
from types import SimpleNamespace
import unittest
import numpy as np
from fine_tuned_client import RightFineTuneClient, resample_rows


class FakeClient:
    def __init__(self):
        self.hook = None
        self.observation = None
        self.grip = 125.0

    def set_observation(self, frames, observation, *, timestamp):
        self.frames, self.observation, self.timestamp = frames, observation, timestamp

    def get_action(self, *, commanded):
        self.commanded = commanded
        self.hook(SimpleNamespace(observation=self.observation))
        # Deliberately permuted dataset order: joint 3,1,2,6,4,5.
        return {"per_key": {"action.right_arm": np.tile([.3,.1,.2,.6,.4,.5],(48,1)),
                            "action.right_gripper": np.full((48,1),self.grip)}}


class FineTuneTests(unittest.TestCase):
    def setUp(self):
        self.client = FakeClient()
        names = [f"joint{i}.pos" for i in (3,1,2,6,4,5)] + ["gripper.pos"]
        info = {"fps":30,"features":{"observation.state":{"names":names},"action":{"names":names}}}
        body = {"state":{"right_arm":[0,6],"right_gripper":[6,7]},
                "action":{"right_arm":[0,6],"right_gripper":[6,7]},
                "video":{"overhead":"scene","wrist_r":"wrist"}}
        self.adapter = RightFineTuneClient(self.client,body,info)
        self.client.hook = self.adapter._capture_trip
        self.adapter.set_observation(
            {"video.overhead":"scene","video.wrist_r":"right","video.wrist_l":"left"},
            {"state.right_arm":np.array([.1,.2,.3,.4,.5,.6]),"state.right_gripper":[25],
             "state.left_arm":np.ones(6),"prompt":"original task"},timestamp=3)

    def test_observation_uses_dataset_order_and_percent_units(self):
        np.testing.assert_array_equal(self.client.observation["state.right_arm"],[.3,.1,.2,.6,.4,.5])
        np.testing.assert_array_equal(self.client.observation["state.right_gripper"],[25])
        self.assertEqual(set(self.client.frames),{"video.overhead","video.wrist_r"})
        self.assertEqual(set(self.client.observation),{"state.right_arm","state.right_gripper","prompt"})

    def test_action_is_reordered_and_percent_grip_is_clamped(self):
        anchor={"q_r_chunk":np.arange(6.),"q_l_chunk":np.zeros(6)}
        reply=self.adapter.get_action(commanded=anchor)
        np.testing.assert_allclose(reply["q_r_chunk"],np.tile([.1,.2,.3,.4,.5,.6,1.0],(48,1)))
        self.assertEqual(set(self.client.commanded),{"q_r_chunk"})
        self.assertEqual(set(reply),{"q_r_chunk"})

    def test_zero_grip_after_negative_prediction(self):
        self.client.grip=-5
        reply=self.adapter.get_action()
        np.testing.assert_allclose(np.asarray(reply["q_r_chunk"])[:,-1],0)

    def test_scalar_gripper_rows_match_column_gripper_rows(self):
        original = self.client.get_action
        def flat_grip(**kwargs):
            reply = original(**kwargs)
            reply["per_key"]["action.right_gripper"] = np.full(48, 50.0)
            return reply
        self.client.get_action = flat_grip
        result = self.adapter.get_action()
        np.testing.assert_allclose(np.asarray(result["q_r_chunk"])[:, -1], 0.5)

    def test_resampling_preserves_physical_speed(self):
        native=np.tile((np.arange(48)/20)[:,None],(1,7))
        converted=resample_rows(native,20,30,48)
        np.testing.assert_allclose(converted[:,0],np.arange(48)/30)
        self.assertLess(converted[-1,0],native[-1,0])

    def test_resampling_refuses_to_invent_future_actions(self):
        with self.assertRaisesRegex(ValueError,"exceed"):
            resample_rows(np.ones((48,7)),30,20,48)

    def test_native_rate_is_unchanged(self):
        original=np.arange(48*7).reshape(48,7)
        np.testing.assert_array_equal(resample_rows(original,30,30,48),original)


if __name__ == "__main__":
    unittest.main()
