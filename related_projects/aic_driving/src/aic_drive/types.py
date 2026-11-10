"""Immutable public controller inputs. No simulator objects cross this boundary."""
from dataclasses import dataclass, asdict
import math
from typing import Literal

@dataclass(frozen=True)
class TrackState:
    track_id: int
    x: float
    y: float
    vx: float
    vy: float
    length: float = 5.0
    width: float = 2.0

@dataclass(frozen=True)
class TrackSample:
    captured_at: float
    state: TrackState

@dataclass(frozen=True)
class EgoState:
    time: float
    x: float
    y: float
    speed: float
    last_acceleration: float = 0.0
    length: float = 5.0
    width: float = 2.0

@dataclass(frozen=True)
class DetectionPacket:
    captured_at: float
    received_at: float
    tracks: tuple[TrackState, ...]

@dataclass(frozen=True)
class ControlDecision:
    acceleration: float
    reason: str
    emergency: bool
    latest_observation_time: float | None
    max_observation_age: float
    candidate_count: int = 0
    prediction_ms: float = 0.0
    degraded: bool = False

@dataclass(frozen=True)
class EpisodeConfig:
    scenario: Literal['follow', 'brake', 'cutin', 'traffic'] = 'brake'
    seed: int = 0
    duration: float = 30.0
    simulation_hz: int = 50
    control_hz: int = 10
    delay: float = 0.2
    delay_mode: Literal['fixed', 'bursty'] = 'fixed'
    position_noise: float = 0.0
    velocity_noise: float = 0.0
    profile: Literal['development', 'holdout'] = 'development'
    def __post_init__(self):
        if self.scenario not in ('follow', 'brake', 'cutin', 'traffic'):
            raise ValueError('Unknown scenario')
        if not isinstance(self.simulation_hz,int) or not isinstance(self.control_hz,int) or self.simulation_hz != 50 or self.control_hz != 10:
            raise ValueError('MVP requires fixed 50 Hz simulation and 10 Hz sensing/control')
        if not all(math.isfinite(v) for v in (self.delay,self.duration,self.position_noise,self.velocity_noise)) or self.delay < 0 or self.duration <= 0 or min(self.position_noise,self.velocity_noise) < 0:
            raise ValueError('Invalid time or noise configuration')
        control_periods = self.duration * self.control_hz
        if not math.isclose(control_periods, round(control_periods), rel_tol=0.0, abs_tol=1e-8):
            raise ValueError('Duration must be an integer multiple of the control period')
        if self.delay_mode not in ('fixed', 'bursty') or self.profile not in ('development','holdout'):
            raise ValueError('Unknown delay mode or scenario profile')
    def to_dict(self):
        return asdict(self)
