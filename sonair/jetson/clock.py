"""Mapping a device clock (UR controller, IMU FIFO timestamp, camera hardware clock) onto the host clock.

Each received packet gives a pair (device time d, host receive time h) with h = a + b d + delay, delay >= 0 and
random. A least-squares fit of h on d is biased by the mean delay and inflated by its spread. The lower envelope is
not: fit the line, then shift it down to the low quantile of the residuals, and refit on the points near that
envelope. b absorbs the crystal drift (tens of ppm) and any error in the nominal tick period, so the nominal
period of a device counter never has to be trusted exactly.
"""
import numpy as np


class ClockMap:
    def __init__(self, q_low=0.02, band_s=0.0005):
        self.q_low, self.band = q_low, band_s
        self.a, self.b, self.d0 = 0.0, 1.0, 0.0
        self.resid_p95 = float("nan"); self.n = 0

    def fit(self, d, h):
        d = np.asarray(d, float); h = np.asarray(h, float)
        ok = np.isfinite(d) & np.isfinite(h)
        d, h = d[ok], h[ok]
        if len(d) < 10: raise ValueError("need at least 10 (device, host) pairs")
        self.d0 = float(d[0]); x = d - self.d0
        b, a = np.polyfit(x, h, 1)
        r = h - (a + b * x)
        a += np.quantile(r, self.q_low)
        for _ in range(2):                                  # refit on the lower envelope
            r = h - (a + b * x)
            keep = r <= np.quantile(r, self.q_low) + self.band
            if keep.sum() >= 10: b, a = np.polyfit(x[keep], h[keep], 1)
            a += np.quantile(h - (a + b * x), self.q_low)
        self.a, self.b, self.n = float(a), float(b), int(len(d))
        self.resid_p95 = float(np.quantile(h - self.to_host(d), 0.95))   # the transport delay above the envelope
        return self

    def to_host(self, d):
        return self.a + self.b * (np.asarray(d, float) - self.d0)

    def drift_ppm(self):
        return (self.b - 1.0) * 1e6

    def summary(self):
        return dict(n=self.n, offset_s=self.a, drift_ppm=round(self.drift_ppm(), 2),
                    delay_above_envelope_p95_ms=round(self.resid_p95 * 1e3, 3))
