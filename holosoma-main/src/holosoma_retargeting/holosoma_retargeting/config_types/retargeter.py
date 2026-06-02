"""Configuration types for retargeter settings."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class FootLockConfig:
    """Configuration for explicit frame-range based foot locking constraints."""

    enable: bool = False
    """Whether to enforce explicit frame-range based foot locking constraints."""

    windows: dict[str, list[tuple[int, int]]] | None = None
    """Per-foot inclusive frame windows for locking.
    Example: {"L_Toe": [(30, 60)], "R_Toe": [(10, 20), (80, 95)]}"""

    z_floor: float = 0.0
    """Floor height used by Z pinning constraints."""

    tolerance: float = 5e-3
    """Tolerance for Z floor pinning constraints."""

    soft: bool = False
    """Use a soft quadratic Z floor penalty instead of hard Z pinning constraints."""

    weight: float = 100.0
    """Weight for soft Z floor pinning when soft is enabled."""


@dataclass(frozen=True)
class StanceConfig:
    """Configuration for stance-aware flat-foot retargeting costs."""

    enable: bool = False
    """Whether to enable stance-aware flat-foot retargeting."""

    ramp_frames: int = 8
    """Frames used to ramp stance costs in and out near contact transitions."""

    min_contact_frames: int = 6
    """Minimum contact segment length retained for stance costs."""

    flat_foot_weight: float = 200.0
    """Weight for reducing stance-side sole marker height spread."""

    sole_ground_weight: float = 50.0
    """Weight for pulling stance-side sole marker mean height to z_floor."""

    root_height_weight: float = 25.0
    """Weight for keeping root height near the initial retargeting height during stance."""

    knee_posture_weight: float = 10.0
    """Weight for keeping stance-side knee near the robot nominal pose."""

    foot_tracking_weight_multiplier: float = 0.15
    """Multiplier for stance-side human foot/toe Laplacian tracking weights."""

    z_floor: float = 0.004
    """Target stance sole marker height."""


@dataclass(frozen=True)
class SelfCollisionConfig:
    """Configuration for self-collision avoidance constraints."""

    enable: bool = False
    """Whether to enforce self-collision constraints."""

    pairs: list[tuple[str, str]] = field(default_factory=list)
    """Body name pairs to check for self-collision.
    Example: [("left_elbow_link", "left_knee_link"), ("left_wrist_yaw_link", "left_knee_link")]"""

    windows: list[tuple[int, int]] | None = None
    """Inclusive frame windows during which self-collision is enforced.
    If None, enforced on all frames.
    Example: [(50, 120)] means only enforce on frames 50..120."""

    tolerance: float = 0.02
    """Minimum distance (meters) to maintain between body pairs."""


@dataclass(frozen=True)
class RetargeterConfig:
    """Configuration for retargeter parameters.

    These parameters control the retargeting optimization process.
    """

    q_a_init_idx: int = -7
    """Index in robot's configuration where optimization variables start.
    -7: starts from floating base, -3: starts from translation of floating base,
    0: starts from actuated DOF, 12: starts from waist, 15: starts from left shoulder"""

    fix_orientation: bool = False
    """When True, fix the floating-base quaternion at its initial value and only optimize
    xyz translation + actuated joint angles. Useful for robots whose body origins are
    co-located at joints=0 (e.g. ch_robot), where unconstrained quaternion optimization
    causes the optimizer to tilt the torso to minimize interaction mesh cost."""

    activate_joint_limits: bool = True
    """Whether to enforce joint limits during retargeting."""

    activate_obj_non_penetration: bool = True
    """Whether to enforce object non-penetration constraints."""

    activate_foot_sticking: bool = True
    """Whether to enforce foot sticking constraints."""

    penetration_tolerance: float = 0.001
    """Tolerance for penetration when enforcing non-penetration constraints."""

    foot_sticking_tolerance: float = 1e-3
    """Tolerance for foot sticking constraints in x, y."""

    foot_lock: FootLockConfig = field(default_factory=FootLockConfig)
    """Configuration for explicit frame-range based foot locking."""

    stance: StanceConfig = field(default_factory=StanceConfig)
    """Configuration for stance-aware flat-foot retargeting."""

    step_size: float = 0.2
    """Trust region for each SQP iteration."""

    visualize: bool = False
    """Whether to visualize the retargeting process."""

    debug: bool = False
    """Whether to enable debug mode."""

    self_collision: SelfCollisionConfig = field(default_factory=SelfCollisionConfig)
    """Configuration for self-collision avoidance."""

    w_nominal_tracking_init: float = 5.0
    """Initial weight for nominal tracking cost."""

    nominal_tracking_tau: float = 1e6
    """Time constant for the nominal tracking cost."""
