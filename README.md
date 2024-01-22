# Quadcopter design made from scratch.

The goal of the project is to create a working model of a quadcopter UAV, which can be
controlled with a Xbox controller. The project utilises the MPU6050 inertial measurement
unit, which combines MEMS accelerometer and gyroscope into a single IC. The combination of
sensor readings from the IMU combined with an adequate mathematical model and a regulating
system allows to make a quadcopter with a stable control system.

More detailed documentation has been placed in the project repository.

## Part list

- Frame: https://www.thingiverse.com/thing:1604440
- 7mm Motor Holder: https://www.thingiverse.com/thing:1675767
- 7x20mm 3.7V DC 46500RPM Motors CCW
- 7x20mm 3.7V DC 46500RPM Motors CW
- Propellers: 55mm
- Transceiver: 2.4GHz NRF24L01
- LiPo Battery: 520mAh 25C 12g Dualsky
- Gyroscope/Accelerometer: MPU6050
- Pressure Sensor: BMP280
- STM32-FC Board (located in the hardware folder)
