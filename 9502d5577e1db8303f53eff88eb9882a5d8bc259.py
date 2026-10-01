"""Minimal contract-shaped hold policy. Replace infer with your trained model."""

import numpy as np


class Policy:
    def __init__(self, checkpoint_dir: str):
        self.checkpoint_dir = checkpoint_dir
        # Load trusted local weights/processors here. No model included in this example.

    def infer(self, obs: dict) -> np.ndarray:
        # Exactly the Arena keys, including two 480x640 RGB uint8 wrist images.
        state = np.concatenate(
            [
                obs["observation.pose.left_hand_root_to_right_hand_root.absolute"],
                obs["observation.joint_states"],
            ]
        )  # 9 values, not dataset observation.state mode flags
        actions = np.zeros((32, 16), dtype=np.float32)
        actions[:, 6] = actions[:, 13] = 1.0  # identity xyzw deltas
        actions[:, 14:16] = state[7:9]  # absolute motor radians
        return actions
