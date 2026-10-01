"""Local Policy(checkpoint_dir).infer(obs) evaluation; no model/network side effects."""

from pathlib import Path
import importlib.util
import time
import numpy as np
from .geometry import pose


def validate_chunk(actions, min_rows=16, *, executed_rows=None):
    if not isinstance(min_rows, int) or min_rows < 1:
        raise ValueError("min_rows must be a positive integer")
    a = np.asarray(actions)
    if a.ndim != 2 or a.shape[1] != 16 or a.shape[0] < min_rows:
        raise ValueError(f"policy must return (N,16) with N>={min_rows}; received {a.shape}")
    if not np.issubdtype(a.dtype, np.number) or np.iscomplexobj(a) or not np.isfinite(a).all():
        raise ValueError("policy actions must be finite numeric values")
    if executed_rows is None:
        executed_rows = len(a)
    if not isinstance(executed_rows, int) or not 1 <= executed_rows <= len(a):
        raise ValueError("executed_rows must be an integer within the returned chunk")
    for row in a[:executed_rows]:
        pose(row[:7])
        pose(row[7:14])
    return a.astype(np.float64)


class HoldPolicy:
    def __init__(self, checkpoint_dir=None):
        pass

    def infer(self, obs):
        a = np.zeros((32, 16), dtype=np.float32)
        a[:, 6] = a[:, 13] = 1
        a[:, 14:16] = obs["observation.joint_states"]
        return a


def load_policy(policy_path, checkpoint_dir):
    path = Path(policy_path).resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    spec = importlib.util.spec_from_file_location("yubi_user_policy", path)
    if spec is None or spec.loader is None:
        raise ValueError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Policy(str(Path(checkpoint_dir).resolve()))


def run_policy(
    env, policy, *, adopt_rows=16, max_calls=None, on_step=None, translation_frame="body"
):
    """Measure simulation task success, not Arena real-robot scores.

    Simulator pauses while infer runs, as described by the public robot contract.
    Translation frame and row frequency are explicit caller choices.
    No dataset-only gripper time shift is applied to live actions.
    """
    if not isinstance(adopt_rows, int) or adopt_rows <= 0:
        raise ValueError("adopt_rows must be a positive integer")
    if max_calls is not None and (not isinstance(max_calls, int) or max_calls <= 0):
        raise ValueError("max_calls must be a positive integer")
    latencies = []
    calls = 0
    rate_limited_steps = 0
    status = "running"
    error = None
    while not env._done and (max_calls is None or calls < max_calls):
        obs = env.observe(images=True)
        start = time.perf_counter()
        try:
            actions = validate_chunk(policy.infer(obs), executed_rows=adopt_rows)
            latencies.append(time.perf_counter() - start)
            # Fail before partially executing a chunk with illegal gripper positions.
            for row in actions[:adopt_rows]:
                env._motor_to_jaw(row[14:16])
            calls += 1
            for action in actions[:adopt_rows]:
                _, _, terminated, truncated, info = env.step_delta(
                    action, translation_frame=translation_frame
                )
                rate_limited_steps += int(any(info["target_rate_limited"]))
                if on_step:
                    on_step(env, info)
                if terminated or truncated:
                    break
        except (ValueError, RuntimeError, FloatingPointError) as exc:
            status, error = "invalid_policy_output", f"{type(exc).__name__}: {exc}"
            break
    if status == "running":
        status = "success" if env.info()["success"] else "horizon" if env._done else "call_limit"
    return {
        **env.info(),
        "status": status,
        "error": error,
        "policy_calls": calls,
        "latency_seconds": latencies,
        "latency_p95_seconds": float(np.percentile(latencies, 95)) if latencies else None,
        "rate_limited_steps": rate_limited_steps,
        "action_hz": env.config.control_hz,
        "translation_frame": translation_frame,
        "adopt_rows": adopt_rows,
        "scoring": "local_sim_task_success_only",
        "camera_calibrated": False,
        "motor_mapping_calibrated": False,
    }
