"""Arm-free YUBI simulation. Set MUJOCO_GL before first environment import."""

__all__ = ["YubiEnv", "SimConfig"]
__version__ = "0.1.0"


def __getattr__(name):
    # Keep CLI import lightweight so headless GL selection precedes MuJoCo import.
    if name in __all__:
        from .env import YubiEnv, SimConfig

        return {"YubiEnv": YubiEnv, "SimConfig": SimConfig}[name]
    raise AttributeError(name)
