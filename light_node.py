from pid import AnalogChannel, PwmChannel


class LightPID:
    def __init__(self, kp, ki, kd, min_out=0.0, max_out=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.reset()

    def reset(self):
        self._accum = 0.0
        self._prev_err = None
        self.last_err = 0.0
        self.term_p = 0.0
        self.term_i = 0.0
        self.term_d = 0.0

    def _clamp_integral(self):
        if self.ki <= 0:
            return
        lower = self.min_out / self.ki
        upper = self.max_out / self.ki
        self._accum = max(lower, min(upper, self._accum))

    def update(self, target, measured, dt):
        err = target - measured
        if self._prev_err is None or dt <= 0:
            rate = 0.0
        else:
            rate = (err - self._prev_err) / dt

        self._prev_err = err
        self.last_err = err
        self._accum += err * dt
        self._clamp_integral()

        self.term_p = self.kp * err
        self.term_i = self.ki * self._accum
        self.term_d = self.kd * rate

        raw_cmd = self.term_p + self.term_i + self.term_d
        return max(self.min_out, min(self.max_out, raw_cmd))


class LightController:
    VALID_MODES = ("off", "manual", "follow", "pid")

    def __init__(self, link, ldr_ch, led_pin, dark_ref, bright_ref, samples=8):
        self.adc = AnalogChannel(link, ldr_ch)
        self.pwm = PwmChannel(link, led_pin)
        self.dark_ref = dark_ref
        self.bright_ref = bright_ref
        self.samples = samples

        self.mode = "off"
        self.setpoint = 50.0
        self.gamma = 1.0
        self.pid = LightPID(1.0, 0.0, 0.0)

        self.last_raw = 0
        self.last_pct = 0.0
        self.control_out = 0.0

    def sample_raw(self):
        return self.adc.read_avg(self.samples)

    def raw_to_pct(self, raw_val):
        span = self.bright_ref - self.dark_ref
        if span == 0:
            return 0.0
        pct = ((raw_val - self.dark_ref) / span) * 100.0
        return max(0.0, min(100.0, pct))

    def sample_pct(self):
        self.last_raw = self.sample_raw()
        self.last_pct = self.raw_to_pct(self.last_raw)
        return self.last_pct

    def apply_pct(self, duty_pct):
        bounded = max(0.0, min(100.0, duty_pct))
        self.pwm.duty_u16(round(bounded * 655.35))

    def current_duty_pct(self):
        return self.pwm.duty_u16() / 655.35

    def set_mode(self, new_mode):
        if new_mode not in self.VALID_MODES:
            raise ValueError(f"Unsupported mode: {new_mode}")
        if new_mode == "pid" and self.mode != "pid":
            self.pid.reset()
        self.mode = new_mode
        if new_mode == "off":
            self.apply_pct(0.0)

    def tick(self, dt):
        self.sample_pct()
        if self.mode == "off":
            self.apply_pct(0.0)
        elif self.mode == "follow":
            self.apply_pct(self.last_pct)
        elif self.mode == "pid":
            self.control_out = self.pid.update(self.setpoint, self.last_pct, dt)
            shaped = 100.0 * ((self.control_out / 100.0) ** self.gamma)
            self.apply_pct(shaped)


