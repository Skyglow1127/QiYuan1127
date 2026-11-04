"""Offline C2 proposals, integer propagation, and an exact discrete stopping tail.

Floating planning is advisory. Every accepted segment passes the unchanged
integer golden's full Bernstein/quantization checks. No state snapping occurs.
"""
from fractions import Fraction
import numpy as np
from sim.player_reference import (SCALE, Bounds, State, PlayerFault, advance,
                                  encode_q32, round_even, ceil, encode_can, to_q16)
from .trajectory import C2Tracker, MotionBounds


def physical(vector):
    return np.asarray(vector, dtype=float)/SCALE


def physical_jerk(vector):
    # Q32 integers exactly round-trip through binary64 within this checked range.
    result = [float(Fraction(x, SCALE)) for x in vector]
    if tuple(encode_q32(x) for x in result) != tuple(vector):
        raise ValueError('jerk cannot losslessly round-trip through bank JSON')
    return result


class IntegerPlanner:
    def __init__(self, state: State, bounds: MotionBounds, *, clock_hz, response_time_s):
        self.state, self.bounds = state, bounds
        self.integer_bounds = Bounds.from_physical(**{
            name: getattr(bounds, name).tolist() for name in bounds.__dataclass_fields__})
        self.clock_hz, self.response_time_s = clock_hz, response_time_s
        self.wire = encode_can(tuple(to_q16(q,l,u) for q,l,u in
            zip(state.q,self.integer_bounds.lower,self.integer_bounds.upper)), self.integer_bounds)

    def checked_step(self, state, jerk, duration, previous_wire):
        step = advance(state, jerk, duration, self.integer_bounds,
                       expected_start=state, clock_hz=self.clock_hz)
        wire = encode_can(step.q16, self.integer_bounds, previous_wire)
        return step.next_state, wire

    def propose(self, target, duration, *, braking=False):
        # Reconstruct from the actual integer endpoint, retaining ALL q/v/a.
        tracker = C2Tracker(physical(self.state.q), self.bounds, self.response_time_s,
                            velocity=physical(self.state.v), acceleration=physical(self.state.a))
        proposal = tracker.advance(target, duration/self.clock_hz, braking=braking)
        jerk = tuple(encode_q32(x) for x in proposal['j'])
        next_state, wire = self.checked_step(self.state, jerk, duration, self.wire)
        self.state, self.wire = next_state, wire
        return physical_jerk(jerk)

    def exact_stop_pair(self, d1, d2):
        """Solve two RNE-integrated segments for EXACT v=a=0, or reject.

        Search around the rational ideal intermediate acceleration. For each
        candidate, propagate RNE(v,a) and solve the second jerk; the endpoint
        must be exactly zero before any bound certification. This finite search
        is deliberately incomplete: failure asks for another bounded brake
        tick, never relaxed thresholds. No endpoint is assigned zero by hand.
        """
        h1, h2 = Fraction(d1,self.clock_hz), Fraction(d2,self.clock_hz)
        radius = ceil(2/(h1+h2))+4
        if radius > 20000:
            raise ValueError('execution rate exceeds offline exact-stop search budget')
        first, second = [], []
        for axis, (v,a) in enumerate(zip(self.state.v,self.state.a)):
            ideal = round_even(-(2*v+a*h1)/(h1+h2))
            found = None
            # Zero offset first; outward deterministic search for RNE lattice.
            for offset in [0]+[s*i for i in range(1,radius+1) for s in (-1,1)]:
                j1 = round_even((ideal+offset-a)/h1)
                a1 = round_even(a+j1*h1)
                v1 = round_even(v+a*h1+j1*h1*h1/2)
                j2 = round_even(-Fraction(a1)/h2)
                if max(abs(j1),abs(j2)) > self.integer_bounds.jerk[axis]:
                    continue
                if round_even(a1+j2*h2) == 0 and round_even(v1+a1*h2+j2*h2*h2/2) == 0:
                    # Check per-axis hull too: another lattice candidate may be feasible.
                    b = Bounds(*( (getattr(self.integer_bounds,k)[axis],)
                                 for k in self.integer_bounds.__dataclass_fields__))
                    s = State((self.state.q[axis],),(v,),(a,))
                    try:
                        s = advance(s,(j1,),d1,b,clock_hz=self.clock_hz).next_state
                        advance(s,(j2,),d2,b,clock_hz=self.clock_hz)
                    except PlayerFault:
                        continue
                    found = (j1,j2)
                    break
            if found is None:
                raise PlayerFault('no feasible exact two-tick integer stop from current state')
            first.append(found[0]);second.append(found[1])
        middle, wire = self.checked_step(self.state, first, d1, self.wire)
        end, wire = self.checked_step(middle, second, d2, wire)
        if any((*end.v,*end.a)):
            raise AssertionError('integer stop solver failed exact terminal constraint')
        jerks = [physical_jerk(first), physical_jerk(second)]
        self.state, self.wire = end, wire
        return jerks

    def stop(self, durations):
        """Bounded damping until a certified exact pair fits remaining capacity."""
        tail = []
        for i in range(len(durations)-1):
            try:
                return tail+self.exact_stop_pair(durations[i],durations[i+1])
            except PlayerFault:
                if i == len(durations)-2:
                    break
            tail.append(self.propose(physical(self.state.q),durations[i],braking=True))
        raise PlayerFault('no certified exact stop within configured bank/tail budget')
