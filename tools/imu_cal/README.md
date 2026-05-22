# IMU calibration host pipeline (Phase 2.5, Issue #145)

Tedaldi (imu_tk) offline calibration tooling for the spike-nx LSM6DSL.
Generates `imu_cal.txt` for `/mnt/flash` on the Hub from a BTSensor
IMU_CAP capture.

## Stage overview

```
ImuViewer Capture tab        (.bin: 27 B IMU_CAP frames from BT SPP)
        │
        ▼
bin_to_ascii.py              accel.txt + gyro.txt + meta.txt
        │
        ▼
run_imu_tk.sh                test_imu_acc.calib + test_imu_gyro.calib
  (docker → ubuntu-imu_tk)
        │
        ▼
imu_tk_output_to_cfg.py      imu_cal.txt   (schema_version=1)
        │
        ▼
zmodem / dfu / nsh tee   →   /mnt/flash/imu_cal.txt on the Hub
        │
        ▼
reboot                       drivebase_imu_cal.c reads at startup
```

## Capture session protocol

1. Drivebase must be stopped (`drivebase stop`) so the layer-2 ODR=833
   force does not race the capture session.
2. Hub: `btsensor _imu_cap start [duration_sec]` (or ImuViewer Capture
   tab → Start).  Default duration is unbounded; for a Tedaldi-quality
   session aim for ~240 s with ~10 static poses of ~3 s each plus
   short hand rotations between them.
3. ImuViewer logs the raw 27 B frame stream straight to a `.bin` file.
   Sessions with `seq drop > 0` (any gap) are rejected on save.
4. Hub: `btsensor _imu_cap stop` (auto-fires when the `duration_sec`
   timer expires).

## Pipeline commands

```bash
# 1. parse the .bin to imu_tk ASCII + session meta
./bin_to_ascii.py path/to/capture.bin --out-dir session_dir

# 2. run the imu_tk calibrator (writes .calib files into session_dir)
./run_imu_tk.sh session_dir

# 3. fold .calib + meta.txt into the Hub-side imu_cal.txt
./imu_tk_output_to_cfg.py --session-dir session_dir

# 4. inspect (no edits — re-run the pipeline if something is off)
cat session_dir/imu_cal.txt
```

The diagonal of `gyro_M_x1000` / `accel_M_x1000` should land within
~2 % of 1000 (= sensor matches its nominal sensitivity).  Larger
deviations trigger a warning and usually mean the capture session
needs to be redone with better static intervals.

## Hub-side install

```bash
# from the host
sx -k session_dir/imu_cal.txt        # on host side of NSH `rz`
```

In NSH:

```
nsh> rz                              # zmodem receive on /mnt/flash
nsh> mv /mnt/flash/imu_cal.txt /mnt/flash/imu_cal.txt
nsh> reboot                          # drivebase loads at startup
```

## File reference

| File | Role |
|---|---|
| `bin_to_ascii.py` | 27 B frame parser → imu_tk `importAsciiData()` format. Validates seq monotonicity and fsr_*_idx invariance. Stdlib only. |
| `run_imu_tk.sh` | Docker wrapper for `ghcr.io/owhinata/ubuntu-imu_tk`. Override image with `IMU_TK_IMAGE=...`. |
| `imu_tk_output_to_cfg.py` | Reads imu_tk `.calib` (T, K, B per triad) + `meta.txt`, computes `M = T × K`, dimensionalises to LSB-space, x1000 fixed-point. Requires numpy. |

## Wire format reference

See `apps/btsensor/btsensor_wire.h` and
`apps/btsensor/btsensor_imu_cap_mode.c` for the on-device side.
`schema_version` of `imu_cal.txt` is hard-checked by the Hub loader;
incompatible files are rejected and the daemon falls back to Identity
+ zero-bias.
