import queue
import sys
import threading
import time

from light_node import LightController
from pid import FirmataLink

SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200
ADC_SAMPLE_MS = 5

LDR_ADC_CHANNEL = 1  # A1
PWM_LED_PIN = 9      # D9

# 16-bit (0..65535) calibration anchors for LDR light percentage
DARK_REF = 9675
BRIGHT_REF = 56644
AVG_SAMPLES = 16

TARGET_PCT = 50.0
GAMMA_EXP = 1.0
GAIN_KP = 0.5
GAIN_KI = 0.2
GAIN_KD = 0.05

INITIAL_MODE = "pid"
LOOP_INTERVAL_S = 0.05
LOG_INTERVAL_S = 1.0

USAGE = """Available commands:
  raw | pct                     read LDR raw u16 or scaled %
  set <0-100>                   set manual LED duty %
  mode off|manual|follow|pid    switch controller operating mode
  sp <0-100>                    set target light %
  kp|ki|kd <value>              adjust PID tuning gains
  gamma <0.5-5>                 set gamma curve exponent
  n <1-64>                      set moving-average sample count
  cal | cal dark | cal bright   view or capture calibration anchors
  tele on|off                   toggle periodic telemetry
  status                        print full state summary
  quit                          turn LED off and exit"""


def bound(val, lo, hi):
    return max(lo, min(hi, val))


def format_telemetry(ctrl, start_time):
    elapsed = time.monotonic() - start_time
    return (
        f"t={elapsed:.1f} raw={ctrl.last_raw} pct={ctrl.last_pct:.1f} "
        f"sp={ctrl.setpoint:.1f} duty={ctrl.current_duty_pct():.1f} mode={ctrl.mode}"
    )


def format_status(ctrl, link, runtime):
    p = ctrl.pid
    return "\n".join([
        f"mode={ctrl.mode} raw={ctrl.last_raw} pct={ctrl.last_pct:.1f} "
        f"duty={ctrl.current_duty_pct():.1f}% duty_u16={ctrl.pwm.duty_u16()}",
        f"cal dark={ctrl.dark_ref} bright={ctrl.bright_ref} n={ctrl.samples} "
        f"adc_bits={ctrl.adc.bits} pwm_bits={ctrl.pwm.bits} firmata={link.fw_version}",
        f"pid sp={ctrl.setpoint:.1f} kp={p.kp:.3f} ki={p.ki:.3f} kd={p.kd:.3f} "
        f"e={p.last_err:.2f} P={p.term_p:.2f} I={p.term_i:.2f} D={p.term_d:.2f} "
        f"u={ctrl.control_out:.1f} gamma={ctrl.gamma:.2f}",
        f"timing period_ms={LOOP_INTERVAL_S * 1000:.0f} "
        f"dt_ms={runtime['dt'] * 1000:.1f}",
    ])


def measure_anchor(ctrl):
    ctrl.set_mode("off")
    time.sleep(0.5)
    return ctrl.sample_raw()


def dispatch_command(raw_line, ctrl, link, runtime):
    tokens = raw_line.strip().lower().split()
    if not tokens:
        return

    print("> " + " ".join(tokens))
    cmd = tokens[0]
    arg = tokens[1] if len(tokens) > 1 else ""

    try:
        if cmd in ("help", "?"):
            print(USAGE)
        elif cmd == "raw":
            print(ctrl.sample_raw())
        elif cmd == "pct":
            print(f"{ctrl.sample_pct():.1f}")
        elif cmd == "set":
            ctrl.set_mode("manual")
            ctrl.apply_pct(float(arg))
            print(f"brightness={ctrl.current_duty_pct():.1f}% duty_u16={ctrl.pwm.duty_u16()}")
        elif cmd == "mode":
            ctrl.set_mode(arg)
            print(f"mode={ctrl.mode}")
        elif cmd == "sp":
            ctrl.setpoint = bound(float(arg), 0.0, 100.0)
            print(f"sp={ctrl.setpoint:.1f}")
        elif cmd in ("kp", "ki", "kd"):
            setattr(ctrl.pid, cmd, float(arg))
            print(f"{cmd}={float(arg):.3f}")
        elif cmd == "gamma":
            ctrl.gamma = bound(float(arg), 0.5, 5.0)
            print(f"gamma={ctrl.gamma:.2f}")
        elif cmd == "n":
            ctrl.samples = int(bound(int(arg), 1, 64))
            print(f"n={ctrl.samples}")
        elif cmd == "cal":
            if arg == "dark":
                ctrl.dark_ref = measure_anchor(ctrl)
            elif arg == "bright":
                ctrl.bright_ref = measure_anchor(ctrl)
            print(f"DARK={ctrl.dark_ref} BRIGHT={ctrl.bright_ref}")
        elif cmd == "tele":
            runtime["tele"] = arg != "off"
            runtime["next_tele"] = time.monotonic() + LOG_INTERVAL_S
            print("tele=" + ("on" if runtime["tele"] else "off"))
        elif cmd == "status":
            print(format_status(ctrl, link, runtime))
        elif cmd in ("quit", "exit", "q"):
            runtime["running"] = False
        else:
            print("? unknown command, type help")
    except ValueError:
        print("? bad value, type help")

    sys.stdout.flush()


def _stdin_worker(out_queue):
    for line in sys.stdin:
        out_queue.put(line)
    out_queue.put("quit")


def run():
    link = FirmataLink(SERIAL_PORT, SERIAL_BAUD, ADC_SAMPLE_MS)
    ctrl = LightController(
        link,
        LDR_ADC_CHANNEL,
        PWM_LED_PIN,
        DARK_REF,
        BRIGHT_REF,
        AVG_SAMPLES,
    )
    ctrl.setpoint = TARGET_PCT
    ctrl.gamma = GAMMA_EXP
    ctrl.pid.kp = GAIN_KP
    ctrl.pid.ki = GAIN_KI
    ctrl.pid.kd = GAIN_KD
    ctrl.set_mode(INITIAL_MODE)

    cmd_queue = queue.Queue()
    threading.Thread(target=_stdin_worker, args=(cmd_queue,), daemon=True).start()

    t_start = time.monotonic()
    runtime = {
        "running": True,
        "tele": True,
        "dt": LOOP_INTERVAL_S,
        "next_tele": t_start + LOG_INTERVAL_S,
    }
    print(f"Light node ready on {SERIAL_PORT} (firmata {link.fw_version})", flush=True)

    prev_tick = t_start
    next_tick = t_start

    try:
        while runtime["running"]:
            while not cmd_queue.empty():
                dispatch_command(cmd_queue.get(), ctrl, link, runtime)

            now = time.monotonic()
            if now >= next_tick:
                runtime["dt"] = now - prev_tick
                prev_tick = now
                ctrl.tick(runtime["dt"])
                next_tick += LOOP_INTERVAL_S
                if next_tick < now:
                    next_tick = now + LOOP_INTERVAL_S

            if runtime["tele"] and now >= runtime["next_tele"]:
                print(format_telemetry(ctrl, t_start), flush=True)
                runtime["next_tele"] += LOG_INTERVAL_S

            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.apply_pct(0.0)
        time.sleep(0.05)
        link.close()


if __name__ == "__main__":
    run()


