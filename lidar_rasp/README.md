# lidar_rasp

Raspberry Pi-oriented copy of the LD19 LiDAR tooling.

## Files

- `lidar_reader.py`: Pi-tuned reader (supports `LIDAR_PORT`, prefers Linux serial paths).
- `test_lidar.py`: serial detection and connectivity test utility.
- `run_lidar_headless.py`: non-GUI runtime for SSH/systemd deployments.
- `visualize_lidar.py`: entrypoint that runs the existing visualization stack.
- `lidar_rasp.service`: example systemd unit.

## Quick Start (Pi)

1. Install dependencies:

   ```bash
   cd ~/Capstone/lidar_rasp
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```

2. Detect your LiDAR port:

   ```bash
   python test_lidar.py
   ```

3. Run headless:

   ```bash
   export LIDAR_PORT=/dev/ttyUSB0
   python run_lidar_headless.py --log-every 10
   ```

4. Optional visualization (if Pi desktop/X forwarding is available):

   ```bash
   python visualize_lidar.py --port /dev/ttyUSB0
   ```

## systemd (optional)

Copy and enable service:

```bash
sudo cp lidar_rasp.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable lidar_rasp
sudo systemctl start lidar_rasp
sudo systemctl status lidar_rasp
```

Adjust `User`, `WorkingDirectory`, and `LIDAR_PORT` in `lidar_rasp.service` first.
