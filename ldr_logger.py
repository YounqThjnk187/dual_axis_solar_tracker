
import argparse
import csv
import time
from datetime import datetime
import serial

BAUD_RATE = 115200
PHASES = [
    "DARK",
    "TOP_LEFT",
    "TOP_RIGHT",
    "BOTTOM_LEFT",
    "BOTTOM_RIGHT",
]

def main():
    parser = argparse.ArgumentParser(
        description="ESP32 4-LDR data logger"
    )
    parser.add_argument("--port", required=True,
                        help="Serial port, e.g. COM5")
    parser.add_argument("--baud", type=int,
                        default=BAUD_RATE)
    parser.add_argument("--duration", type=float,
                        default=10,
                        help="Duration of each phase (seconds)")
    args = parser.parse_args()

    filename = (
        "ldr_data_" +
        datetime.now().strftime("%Y%m%d_%H%M%S") +
        ".csv"
    )

    print(f"Connecting to {args.port}...")

    try:
        ser = serial.Serial(
            args.port,
            args.baud,
            timeout=1
        )
    except serial.SerialException as e:
        print(f"Cannot open serial port: {e}")
        return

    time.sleep(2)
    ser.reset_input_buffer()

    columns = [
        "phase",
        "pc_timestamp",
        "phase_elapsed_s",
        "esp_millis",
        "TL_raw",
        "TR_raw",
        "BL_raw",
        "BR_raw",
    ]

    total_samples = 0

    try:
        with open(filename, "w", newline="") as csvfile:
            writer = csv.DictWriter(
                csvfile,
                fieldnames=columns
            )
            writer.writeheader()

            for phase in PHASES:
                input(
                    f"\nPrepare: {phase}. "
                    f"Press ENTER to start..."
                )

                print(
                    f"Recording {phase} "
                    f"for {args.duration} seconds..."
                )

                start = time.time()
                count = 0

                while time.time() - start < args.duration:
                    line = ser.readline().decode(
                        "utf-8",
                        errors="ignore"
                    ).strip()

                    if not line.startswith("DATA,"):
                        continue

                    parts = line.split(",")

                    if len(parts) != 6:
                        continue

                    try:
                        esp_millis = int(parts[1])
                        tl = int(parts[2])
                        tr = int(parts[3])
                        bl = int(parts[4])
                        br = int(parts[5])
                    except ValueError:
                        continue

                    elapsed = time.time() - start

                    writer.writerow({
                        "phase": phase,
                        "pc_timestamp": datetime.now().isoformat(
                            timespec="milliseconds"
                        ),
                        "phase_elapsed_s": round(elapsed, 3),
                        "esp_millis": esp_millis,
                        "TL_raw": tl,
                        "TR_raw": tr,
                        "BL_raw": bl,
                        "BR_raw": br,
                    })

                    count += 1
                    total_samples += 1

                csvfile.flush()
                print(f"Captured {count} samples for {phase}.")

    except KeyboardInterrupt:
        print("\nRecording interrupted by user.")

    finally:
        ser.close()

    print("\nRecording completed.")
    print(f"Total samples: {total_samples}")
    print(f"CSV saved as: {filename}")


if __name__ == "__main__":
    main()
