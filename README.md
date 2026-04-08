# B31DG-H00422502-Assignment3-FreeRTOS Implementation

## Overview
This project contains my Assignment 3 FreeRTOS implementation for the real-time scheduling laboratory on ESP32-WROOM.  
It includes periodic tasks A, B, AGG, C, and D, one sporadic task S, and a monitor task for timing statistics and final reporting. 

## Files
- `CMakeLists.txt` - project build file  
- `sdkconfig` - project configuration set using `idf.py menuconfig`  
- `main/main.c` - main task and scheduling logic  
- `main/monitor.c` / `main/monitor.h` - timing monitor  
- `main/workkernel.c` - provided workload function

## Configuration
The `sdkconfig` file contains the project settings configured in `idf.py menuconfig`, including:
- FreeRTOS runs on the first core only
- CPU frequency set to 240 MHz
- watchdog-related settings adjusted for development/testing 

## Pin Mapping
- `SYNC` → GPIO4
- `IN_A` → GPIO16
- `IN_B` → GPIO17
- `IN_S` → GPIO18
- `IN_MODE` → GPIO19
- `ACK_A` → GPIO21
- `ACK_B` → GPIO22
- `ACK_AGG` → GPIO23
- `ACK_C` → GPIO25
- `ACK_D` → GPIO26
- `ACK_S` → GPIO27

## Build and Run
Open the **ESP-IDF 5.5 Command Prompt**, go to the project folder, and run:

```bash
idf.py build flash monitor
