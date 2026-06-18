# esp32-controller

A quadcopter flight controller written from scratch for the ESP32, built incrementally on real hardware. No flight-controller framework, no sensor libraries — every layer (I2C chip driver, sensor fusion, control stack, real-time scheduling) is implemented directly to understand how a flight controller actually works from first principles.

The end goal is a drone that estimates its own attitude and holds stable control on real hardware, as the foundation for later autonomous, GPS-denied navigation.

## Status

The full sensing and timing pipeline runs on hardware at a deterministic 200 Hz: the IMU is read, calibrated, and fused into a live attitude estimate on a fixed real-time loop. The control stack (a cascaded PID architecture) is complete and tested in simulation but is not yet wired to the live sensor data — that integration is the next milestone.

In short: the drone knows its orientation in real time. It doesn't yet act on it.

## What works

**Sensor driver (`mpu6050.c`)** — a from-scratch I2C driver for the MPU-6050 (GY-521). Device probe via `WHO_AM_I`, wake from sleep, a single-transaction 14-byte burst read of the accelerometer and gyroscope, and scaling from raw 16-bit counts to physical units (m/s², rad/s). I2C register read/write primitives are built directly on the ESP-IDF command-link API; the bus is owned by `main`, not the driver, so additional I2C devices can share it later.

**Bias calibration** — a zero-rate startup calibration averages a second of samples while the board is stationary and stores per-axis accelerometer and gyroscope bias, subtracted from every subsequent reading. This corrects the substantial factory bias typical of cheap GY-521 boards (the raw chip read gravity ~18% high before calibration).

**Attitude estimation (`imu.c`)** — a complementary filter fuses accelerometer and gyroscope into roll, pitch, and yaw. The accelerometer provides an absolute, drift-free gravity reference; the gyroscope provides clean short-term rates. The two are blended so the gyro is trusted on fast motion and the accelerometer slowly corrects long-term drift. The accelerometer-derived angle formulas were derived from the rotation matrices rather than copied.

**Real-time loop (`main.c`)** — a FreeRTOS architecture gives a deterministic 200 Hz control tick. A hardware `esp_timer` fires every 5 ms and does nothing but signal a high-priority control task via a task notification; the task wakes, runs the read → fuse → log pipeline, and blocks again. This keeps blocking I2C out of the timer context and guarantees a fixed, jitter-free `dt` for integration.

**Control stack (`pid.c`, `mixer.c`, `control.c`)** — a cascaded PID controller: altitude, roll/pitch angle-and-rate cascades, and a yaw-rate loop, with conditional-integration anti-windup and derivative-on-measurement. A mixer maps the four control outputs to per-motor thrusts for an X-configuration quad. Unit-agnostic, NED coordinate convention, validated in simulation. Tuned for realistic FPV-class parameters (~0.25 kg, 4:1 thrust-to-weight).

## Architecture

```
main.c        I2C bus init, FreeRTOS task + 200 Hz timer, pipeline glue
  |
  v
mpu6050.c     I2C chip driver: raw registers -> calibrated physical units
  |
  v
imu.c         complementary filter: accel + gyro -> roll / pitch / yaw
  |
  v
control.c     cascaded PID: attitude + setpoints -> control commands   [not yet wired to live data]
  |
  v
mixer.c       control commands -> per-motor thrusts
```

Each layer has a single responsibility and a clean boundary. The chip driver knows nothing about the drone; the filter knows nothing about the specific chip; the control stack is independent of how state is measured.

## Hardware

- **MCU**: ESP32 DevKit V1
- **IMU**: GY-521 (MPU-6050), I2C — SDA on GPIO 21, SCL on GPIO 22, 400 kHz, powered at 3.3 V
- **Planned**: GY-87 (MPU-6050 + QMC5883L magnetometer + BMP180 barometer) for absolute heading and altitude

## Build

ESP-IDF v5.3.

```
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Known limitations

- **Yaw drifts.** With no magnetometer, integrated yaw has no absolute reference and slowly drifts. This is accepted for now because the controller uses a yaw *rate* setpoint, not an absolute heading. A magnetometer (via the planned GY-87) will fix it.
- **Gyro range.** Currently at the ±250°/s default, which saturates under fast hand rotation. A wider range is a one-register change when needed.
- **Control loop is open.** The PID stack does not yet receive live attitude — closing that loop is the next step.

## Next steps

1. Configure a wider gyro range to avoid saturation during real motion.
2. Wire the live attitude estimate from `imu.c` into `control_update` — closing the loop from sensor to controller for the first time on hardware.
3. Add PWM motor output (props off) and verify the mixer commands respond correctly to attitude changes.
4. Add the GY-87 for absolute heading (magnetometer) and altitude (barometer), feeding the yaw and altitude loops that currently have no real input.
5. Build toward autonomous, GPS-denied navigation and target tracking.

## Notes

This is a learning project built from first principles — derive, implement, verify on hardware, one layer at a time. The commit history follows that path.

