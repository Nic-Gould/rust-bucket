# rust-bucket

This repo currently contains my attempts to write a driver for the MPU9250 in Rust. This work is based primarily on the Kris Winer 9250 driver for arduino, and re-written for rust. As the project progresses I'm hoping to re-write more of it in the style of the IMU example given in the ESP32::SVC crate (I think).

Once the MPU driver is complete, I'm planning to use it as the basis for a drone flight controller (FC) based on the ESP32, also written in rust. Most commonly available FCs for hobbyist use are based on STM32 microcontrollers (ARM), using C++. The ESP32 presents an interesting target platform due to its relatively high performance/cost compared to STM32, easily usable development boards, and integrated Wifi, PWM , ADCs DACs, and a range of other potentially useful modules.
