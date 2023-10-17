```
 _____                                 _      _  _     
/  ___|                               | |    (_)| |    
\ `--.   ___  _ __   ___   ___   _ __ | |     _ | |__  
 `--. \ / _ \| '_ \ / __| / _ \ | '__|| |    | || '_ \ 
/\__/ /|  __/| | | |\__ \| (_) || |   | |____| || |_) |
\____/  \___||_| |_||___/ \___/ |_|   \_____/|_||_.__/ 
                 ···   ···                   
>  Commonly used I2C , SPI device libraries
                                                      
```

[![Arduino CI](https://github.com/lewisxhe/SensorLib/actions/workflows/arduino_ci.yml/badge.svg)](https://github.com/lewisxhe/SensorLib/actions/workflows/arduino_ci.yml)
[![PlatformIO CI](https://github.com/lewisxhe/SensorLib/actions/workflows/pio.yml/badge.svg)](https://github.com/lewisxhe/SensorLib/actions/workflows/pio.yml)
[![arduino-library-badge](https://www.ardu-badge.com/badge/SensorLib.svg?)](https://www.ardu-badge.com/SensorLib)
![Build with PlatformIO](https://img.shields.io/badge/Build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)

[![LICENSE](https://img.shields.io/github/license/lewisxhe/SensorLib)](https://github.com/lewisxhe/SensorLib/blob/master/LICENSE)
[![ISSUES](https://img.shields.io/github/issues/lewisxhe/SensorsLib)](https://github.com/lewisxhe/SensorsLib/issues)
[![FROK](https://img.shields.io/github/forks/lewisxhe/SensorsLib)](https://github.com/lewisxhe/SensorsLib/graphs/contributors)
[![STAR](https://img.shields.io/github/stars/lewisxhe/SensorsLib)](https://github.com/lewisxhe/SensorsLib/stargazers)
[![releases](https://img.shields.io/github/release/lewisxhe/SensorsLib)](https://github.com/lewisxhe/SensorLib/releases)

![PCF8563](https://img.shields.io/badge/PCB8563-GREEN)
![PCF85063](https://img.shields.io/badge/PCF85063-GREEN)
![HYM8563](https://img.shields.io/badge/HYM8563-GREEN)
![QMI8658](https://img.shields.io/badge/QMI8658-blue)
![BMM150](https://img.shields.io/badge/BMM150-blue)
![QMC6310](https://img.shields.io/badge/QMC6310-blue)
![BMA423](https://img.shields.io/badge/BMA423-blue)
![BHI260AP](https://img.shields.io/badge/BHI260AP-blue)
![XL9555](https://img.shields.io/badge/XL9555-yellow)
![DRV2605](https://img.shields.io/badge/DRV2605-teal)
![CM32181](https://img.shields.io/badge/CM32181-brown)
![LTR553](https://img.shields.io/badge/LTR553-brown)
![FT6X36](https://img.shields.io/badge/FT6X36-red)
![CST816T](https://img.shields.io/badge/CST816T-red)
![CST226SE](https://img.shields.io/badge/CST226SE-red)
![CHSC5816](https://img.shields.io/badge/CHSC5816-red)
![GT911](https://img.shields.io/badge/GT911-red)



Support list:

| Sensor          | Description              | I2C | SPI |
| --------------- | ------------------------ | --- | --- |
| PCF8563/HYM8563 | Real-time clock          | ✔️   | ❌   |
| PCF85063        | Real-time clock          | ✔️   | ❌   |
| QMI8658         | IMU                      | ✔️   | ✔️   |
| BHI260AP        | IMU                      | ✔️   | ✔️   |
| QMC6310         | Magnetic Sensor          | ✔️   | ❌   |
| BMM150          | Magnetic Sensor          | ✔️   | ❌   |
| XL9555          | I/O expander             | ✔️   | ❌   |
| BMA423          | Accelerometer            | ✔️   | ❌   |
| DRV2605         | Haptic Driver            | ✔️   | ❌   |
| CM32181         | Ambient Light Sensor     | ✔️   | ❌   |
| LTR553          | Light & Proximity Sensor | ✔️   | ❌   |
| FT6X36          | Capacitive touch         | ✔️   | ❌   |
| CST816T         | Capacitive touch         | ✔️   | ❌   |
| CST226SE        | Capacitive touch         | ✔️   | ❌   |
| CHSC5816        | Capacitive touch         | ✔️   | ❌   |
| GT911           | Capacitive touch         | ✔️   | ❌   |


