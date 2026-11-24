import time
from pyfirmata2 import Arduino

PORT = "/dev/ttyACM0"


def main():
    board = Arduino(PORT)
    sensor = board.get_pin("a:0:i")
    led = board.get_pin("d:9:o")

    latest = [None]

    def on_analog(value):
        if value is not None:
            latest[0] = float(value)

    sensor.register_callback(on_analog)
    sensor.enable_reporting()
    board.samplingOn(50)

    led.write(0)
    time.sleep(0.5)

    try:
        print("LED is OFF (0). Reading A0...", flush=True)
        for _ in range(15):
            print(f"t={time.monotonic():.3f} | LED=0 | A0={latest[0]}", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        led.write(0)
        board.exit()


if __name__ == "__main__":
    main()


