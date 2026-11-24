"""LSM6DSV16X (SparkFun Qwiic board) on the Jetson's I2C bus: FIFO with hardware timestamps.

Why the FIFO and not the output registers: polling the output registers from Linux user space gives samples stamped
with the host's read time, so OS scheduling jitter becomes timestamp noise and any late read silently drops samples.
In FIFO mode the sensor stores every accelerometer and gyroscope sample with its own timestamp; the host reads in
bursts and nothing is lost or re-stamped.

FIFO word = 1 tag byte + 6 data bytes. Tag (bits 7..3): 0x01 gyroscope, 0x02 accelerometer, 0x04 timestamp.
With timestamp batching every data set, each sample takes the most recent timestamp word.

Register map and sensitivities: ST datasheet DS13510 for the LSM6DSV16X. The addresses below are the ones this
driver uses; `python -m sonair jetson-imu-probe` checks WHO_AM_I (0x70), the sample rate and the noise on the bench
before any campaign, and the ClockMap fit absorbs any error in the nominal timestamp period, so a wrong tick
period shows up as a drift number in the QC report rather than as a timing error in the data.

Bandwidth: one IMU at 480 Hz with accelerometer + gyroscope + timestamp is about 3 words x 7 bytes x 480 Hz = 10 kB/s,
roughly 100 kbit/s on the wire with I2C overhead. A 400 kHz bus therefore carries 2 to 3 IMUs at 480 Hz; put the
others on the second I2C bus, behind a Qwiic multiplexer, or on SPI. Use a differential I2C extender for the cable
run along the arm: plain I2C over a metre of moving cable is unreliable.
"""
import struct, time
import numpy as np

WHO_AM_I, WHO_AM_I_VAL = 0x0F, 0x70
FIFO_CTRL3, FIFO_CTRL4 = 0x09, 0x0A
CTRL1, CTRL2, CTRL3, CTRL6, CTRL8 = 0x10, 0x11, 0x12, 0x15, 0x17
FIFO_STATUS1 = 0x1B
FUNCTIONS_ENABLE = 0x50
FIFO_DATA_OUT_TAG = 0x78

TAG_GYRO, TAG_ACCEL, TAG_TS = 0x01, 0x02, 0x04
ODR_CODE = {120: 0x6, 240: 0x7, 480: 0x8, 960: 0x9}                  # accelerometer and gyroscope ODR and FIFO batch rate
ACC_FS = {2: (0x0, 0.061e-3), 4: (0x1, 0.122e-3), 8: (0x2, 0.244e-3), 16: (0x3, 0.488e-3)}          # g -> (code, g/LSB)
GYR_FS = {250: (0x1, 8.75e-3), 500: (0x2, 17.5e-3), 1000: (0x3, 35e-3), 2000: (0x4, 70e-3), 4000: (0xC, 140e-3)}  # dps
G = 9.80665
TS_LSB_NOMINAL_S = 21.75e-6        # nominal timestamp tick; the exact value is fitted by ClockMap (it appears as drift)


def decode_fifo(raw, acc_fs_g=8, gyr_fs_dps=2000, ts_lsb_s=TS_LSB_NOMINAL_S):
    """raw: bytes, a whole number of 7-byte FIFO words. Returns (t_dev s, accel m/s^2 [n,3], gyro rad/s [n,3], counts).

    Samples are emitted when both an accelerometer and a gyroscope word have been seen under the same timestamp.
    The 32-bit timestamp wraps after about 26 h at the nominal tick; wraps are unwrapped here."""
    sa, sg = ACC_FS[acc_fs_g][1] * G, np.radians(GYR_FS[gyr_fs_dps][1])
    t_out, a_out, g_out = [], [], []
    ts, acc, gyr = None, None, None
    counts = dict(words=0, ts=0, acc=0, gyr=0, other=0, orphan=0)
    wraps, last_raw = 0, None
    for k in range(0, len(raw) - len(raw) % 7, 7):
        tag = raw[k] >> 3; data = raw[k + 1:k + 7]; counts["words"] += 1
        if tag == TAG_TS:
            r = struct.unpack("<I", data[:4])[0]
            if last_raw is not None and r < last_raw: wraps += 1
            last_raw = r
            if acc is not None or gyr is not None: counts["orphan"] += 1
            ts, acc, gyr = (r + wraps * 2 ** 32) * ts_lsb_s, None, None; counts["ts"] += 1
        elif tag == TAG_ACCEL:
            acc = np.array(struct.unpack("<hhh", data)) * sa; counts["acc"] += 1
        elif tag == TAG_GYRO:
            gyr = np.array(struct.unpack("<hhh", data)) * sg; counts["gyr"] += 1
        else:
            counts["other"] += 1
        if ts is not None and acc is not None and gyr is not None:
            t_out.append(ts); a_out.append(acc); g_out.append(gyr); acc, gyr = None, None
    return np.array(t_out), np.array(a_out).reshape(-1, 3), np.array(g_out).reshape(-1, 3), counts


def encode_fifo(t_dev, accel, gyro, acc_fs_g=8, gyr_fs_dps=2000, ts_lsb_s=TS_LSB_NOMINAL_S):
    """inverse of decode_fifo (for tests and the mock device)"""
    sa, sg = ACC_FS[acc_fs_g][1] * G, np.radians(GYR_FS[gyr_fs_dps][1])
    out = bytearray()
    for t, a, g in zip(t_dev, accel, gyro):
        out += bytes([TAG_TS << 3]) + struct.pack("<I", int(round(t / ts_lsb_s)) % 2 ** 32) + b"\x00\x00"
        out += bytes([TAG_GYRO << 3]) + struct.pack("<hhh", *np.clip(np.round(np.asarray(g) / sg), -32768, 32767).astype(int))
        out += bytes([TAG_ACCEL << 3]) + struct.pack("<hhh", *np.clip(np.round(np.asarray(a) / sa), -32768, 32767).astype(int))
    return bytes(out)


class LSM6DSV16X:
    """One sensor on an I2C bus (smbus2). read() returns the FIFO bytes available now, with the host time of the read."""
    def __init__(self, bus=1, addr=0x6B, odr=480, acc_fs_g=8, gyr_fs_dps=2000):
        from smbus2 import SMBus, i2c_msg
        self._msg = i2c_msg
        self.bus, self.addr, self.odr, self.acc_fs_g, self.gyr_fs_dps = SMBus(bus), addr, odr, acc_fs_g, gyr_fs_dps
        who = self.bus.read_byte_data(addr, WHO_AM_I)
        if who != WHO_AM_I_VAL: raise IOError(f"0x{addr:02x} on i2c-{bus}: WHO_AM_I 0x{who:02x}, expected 0x{WHO_AM_I_VAL:02x}")
        code = ODR_CODE[odr]
        w = self.bus.write_byte_data
        w(addr, CTRL3, 0x44)                                   # BDU + register auto-increment
        w(addr, CTRL8, ACC_FS[acc_fs_g][0]); w(addr, CTRL6, GYR_FS[gyr_fs_dps][0])
        w(addr, CTRL1, code); w(addr, CTRL2, code)             # high-performance mode at the chosen ODR
        w(addr, FUNCTIONS_ENABLE, 0x40)                        # timestamp counter on
        w(addr, FIFO_CTRL3, (code << 4) | code)                # batch gyro and accel at the ODR
        w(addr, FIFO_CTRL4, (0x1 << 6) | 0x6)                  # timestamp every batch, FIFO continuous mode

    def read(self):
        st = self.bus.read_i2c_block_data(self.addr, FIFO_STATUS1, 2)
        n = st[0] | ((st[1] & 0x01) << 8)
        out = bytearray()
        while n > 0:
            k = min(n, 36)                                     # 36 words = 252 bytes per I2C transfer
            wr = self._msg.write(self.addr, [FIFO_DATA_OUT_TAG]); rd = self._msg.read(self.addr, 7 * k)
            self.bus.i2c_rdwr(wr, rd); out += bytes(list(rd)); n -= k
        return bytes(out), time.monotonic()


class MockIMU:
    """Stands in for a sensor on the bench: produces FIFO bytes from a function omega(t_host), accel(t_host) with
    sensor noise, a device clock with offset and drift, and bursty reads."""
    def __init__(self, omega_fn, accel_fn, odr=480, drift_ppm=40.0, gyro_noise=0.002, accel_noise=0.02, seed=0, group_delay_s=0.0):
        """group_delay_s: the sensor's filter delay (a sample describes the motion this long before its timestamp)"""
        self.f_w, self.f_a, self.odr, self.gd = omega_fn, accel_fn, odr, group_delay_s
        self.drift, self.rng = drift_ppm * 1e-6, np.random.default_rng(seed)
        self.dev0, self.host0, self.k = 123.456, None, 0
        self.gn, self.an = gyro_noise, accel_noise

    def read_at(self, t_host):
        if self.host0 is None: self.host0 = t_host
        n_due = int((t_host - self.host0) * self.odr) + 1 - self.k           # sample k exists from host0 + k / odr
        if n_due <= 0: return b"", t_host
        ks = np.arange(self.k, self.k + n_due); self.k += n_due
        th = self.host0 + ks / self.odr                                  # true sample instants (host clock)
        td = self.dev0 + (th - self.host0) * (1 + self.drift)           # what the sensor's counter says
        w = np.array([self.f_w(x - self.gd) for x in th]) + self.rng.normal(0, self.gn, (n_due, 3))
        a = np.array([self.f_a(x - self.gd) for x in th]) + self.rng.normal(0, self.an, (n_due, 3))
        return encode_fifo(td, a, w), t_host + float(self.rng.exponential(0.0004))
