import asyncio
from bleak import BleakScanner, BleakClient
from datetime import datetime
from pathlib import Path

CHAR_UUID = "abcdef12-3456-7890-1234-56789abcdef1"
DEVICE_NAME_PREFIX = "Circle"

SAVE_FOLDER = Path(
    "/Users/willowkim/Library/CloudStorage/OneDrive-TheOhioStateUniversity/Circle/Vlog"
)


async def main():
    SAVE_FOLDER.mkdir(parents=True, exist_ok=True)

    print("Scanning for Circle device...")
    devices = await BleakScanner.discover(timeout=10)

    print("\nFound BLE devices:")
    for d in devices:
        print(f"  {d.name} | {d.address}")

    target = next(
        (
            d for d in devices
            if d.name and d.name.startswith(DEVICE_NAME_PREFIX)
        ),
        None
    )

    if not target:
        print("Circle device not found.")
        return

    print(f"\nConnecting to: {target.name}")

    filename = SAVE_FOLDER / (
        f"vlog_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    )

    with open(filename, "w", buffering=1) as f:
        f.write("raw_ble_data\n")

        def handler(_, data: bytearray):
            try:
                line = data.decode(errors="ignore").strip()
            except Exception:
                print("Decode error")
                return

            if not line:
                return

            # Print live BLE data
            print(line)

            # Save raw line
            f.write(line + "\n")
            f.flush()

            # Optional parsing
            if line.startswith("IMU_ACCEL"):
                print("-> Received accelerometer data")

            elif line.startswith("IMU_GYRO"):
                print("-> Received gyro data")

            else:
                print("-> Received VLOG data")

        async with BleakClient(target.address) as client:
            print("Connected successfully")

            await client.start_notify(CHAR_UUID, handler)

            print(f"\nLogging to: {filename}")
            print("Press Ctrl+C to stop.\n")

            while True:
                await asyncio.sleep(1)


try:
    asyncio.run(main())

except KeyboardInterrupt:
    print("\nLogging stopped safely.")