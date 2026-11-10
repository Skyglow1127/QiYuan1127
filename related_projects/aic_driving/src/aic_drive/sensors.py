"""Simulation-time sensing with immutable packets and independent random streams."""
from collections import deque
from dataclasses import replace
import heapq
import numpy as np
from .types import DetectionPacket, TrackSample, TrackState

class DelayedSensor:
    def __init__(self, seed=0, delay=0.2, mode='fixed', position_noise=0., velocity_noise=0., dt=0.1):
        self.delay, self.mode, self.dt = delay, mode, dt
        self.position_noise, self.velocity_noise = position_noise, velocity_noise
        self.noise_rng = np.random.default_rng(np.random.SeedSequence([seed, 101]))
        self.delay_rng = np.random.default_rng(np.random.SeedSequence([seed, 102]))
        self.queue, self.sequence, self.block = [], 0, []
        self.delays = []
    def capture(self, time, states):
        if self.mode == 'fixed':
            lag = self.delay
        else:
            # Exactly 5 high-delay and 15 zero-delay packets per full block.
            # Contiguous high-delay burst, random circular location; same nominal mean.
            if not self.block:
                offset = int(self.delay_rng.integers(20))
                self.block = np.roll(np.r_[np.full(5, 4*self.delay), np.zeros(15)],offset).tolist()
            lag = self.block.pop(0)
        measured = []
        for s in sorted(states, key=lambda s:s.track_id):
            n = self.noise_rng.normal(size=4)
            measured.append(replace(s, x=float(s.x+n[0]*self.position_noise), y=float(s.y+n[1]*self.position_noise),
                                    vx=float(s.vx+n[2]*self.velocity_noise), vy=float(s.vy+n[3]*self.velocity_noise)))
        p = DetectionPacket(float(time), float(time+lag), tuple(measured))
        heapq.heappush(self.queue, (p.received_at, self.sequence, p))
        self.sequence += 1
        self.delays.append(lag)
    def receive(self, time):
        packets=[]
        while self.queue and self.queue[0][0] <= time+1e-9:
            packets.append(heapq.heappop(self.queue)[2])
        return packets

class TrackHistory:
    def __init__(self, window=1.2):
        self.window=window
        self.tracks={}
        self.out_of_order=0
    def ingest(self, packets, now):
        for p in packets:
            if p.received_at > now+1e-9 or p.captured_at > now+1e-9:
                raise ValueError('Attempt to ingest future observations')
            for s in p.tracks:
                history=self.tracks.setdefault(s.track_id, deque())
                if history and p.captured_at <= history[-1].captured_at+1e-9:
                    self.out_of_order += 1
                    continue
                history.append(TrackSample(p.captured_at,s))
                while len(history)>1 and history[0].captured_at < p.captured_at-self.window-1e-9:
                    history.popleft()
    def snapshot(self):
        # Retain stale tracks explicitly; never interpret a missing update as free road.
        return {key:tuple(value) for key,value in self.tracks.items() if value}
