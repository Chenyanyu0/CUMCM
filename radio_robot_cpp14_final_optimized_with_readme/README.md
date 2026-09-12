
# Radio Robot C++14 Final Optimized Version

## 1. Project Introduction

This project is an automatic radio interference source localization and clearing robot program for B Problem 3.

Implemented strategy:

1. Single coverage search
2. Unknown channel detection
3. Direction measurement with measurement error
4. Multi-point localization
5. Localization uncertainty evaluation
6. Adaptive additional measurement
7. Target scheduling
8. Path optimization
9. Automatic clearing

The program communicates with the simulator through HTTP + JSON interface.

Supported simulator commands:

- `/enter`
- `/measure`
- `/clear`
- `/exit`

---

# 2. Required Environment

## Operating System

Recommended:

- Ubuntu 20.04 / 22.04
- WSL Ubuntu is also acceptable

## Compiler

Required:

- g++ with C++14 support
- cmake >= 3.10

Check:

```bash
g++ --version
cmake --version
```

---

## Required Libraries

### libcurl

Used for HTTP communication.

Ubuntu installation:

```bash
sudo apt update
sudo apt install libcurl4-openssl-dev
```

---

### nlohmann/json

The project uses the single-header JSON library.

Required file:

```
include/json.hpp
```

Download:

https://github.com/nlohmann/json/releases

Place the file:

```
radio_robot_cpp14_final_optimized/
│
├── include/
│   └── json.hpp
│
├── src/
└── CMakeLists.txt
```

---

# 3. Configure Robot ID

Before running, modify:

```
src/api.cpp
```

Find:

```cpp
const std::string ROBOT_ID="YOUR_ROBOT_ID";
```

Replace with your competition team ID.

Example:

```cpp
const std::string ROBOT_ID="123456";
```

---

# 4. Build

Enter project directory:

```bash
cd radio_robot_cpp14_final_optimized
```

Create build directory:

```bash
mkdir build
cd build
```

Generate Makefile:

```bash
cmake ..
```

Compile:

```bash
make -j4
```

After successful compilation:

```
build/
└── robot
```

---

# 5. Run Simulator

First:

1. Start the official simulator.
2. Login.
3. Start Problem 3 test.
4. Wait until simulator shows that robot interface is ready.

The simulator default address:

```
http://127.0.0.1:2026
```

Run:

```bash
./robot
```

The robot program will:

1. Enter target area
2. Execute search strategy
3. Detect interference sources
4. Localize targets
5. Clear targets
6. Exit simulator

---

# 6. Algorithm Overview

## Search

Initial search uses:

- Center point
- Six surrounding points

Radius:

```
1150 m
```

forming a hexagonal coverage pattern.

---

## Localization

The program does not treat measurement angle as exact.

Measurement model:

```
theta - 1 degree <= true angle <= theta + 1 degree
```

The localization area is calculated from the intersection of measurement sectors.

---

## Clearing Decision

The program estimates the uncertainty region.

If the uncertainty is small enough:

```
radius <= 20 m
```

the robot clears the target.

Otherwise it performs additional measurement.

---

## Path Planning

The current version includes:

- nearest-neighbor initialization
- 2-opt optimization framework

to reduce robot movement distance.

---

# 7. Output

The program generates:

```
robot.log
```

which records execution information.

---

# 8. Troubleshooting

## Problem: cannot connect to simulator

Check:

- simulator is running
- test has started
- port 2026 is available

---

## Problem: compile error

Missing:

```
json.hpp
```

Solution:

Place nlohmann/json.hpp into:

```
include/json.hpp
```

---

## Problem: robot_id error

Check:

```
src/api.cpp
```

and modify:

```cpp
ROBOT_ID
```

---

# 9. Project Structure

```
radio_robot_cpp14_final_optimized

├── CMakeLists.txt

├── README.md

├── include/
│   ├── api.h
│   ├── geometry.h
│   ├── localization.h
│   ├── planner.h
│   ├── search.h
│   ├── logger.h
│   ├── types.h
│   └── json.hpp

└── src/
    ├── main.cpp
    ├── api.cpp
    ├── geometry.cpp
    ├── localization.cpp
    ├── search.cpp
    ├── planner.cpp
    └── logger.cpp
```
