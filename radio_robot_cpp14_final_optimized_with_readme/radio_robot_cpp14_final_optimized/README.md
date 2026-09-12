
Final optimized version.

Features:
- Simulator HTTP JSON interface
- Single coverage search
- Unknown channel management
- +-1 degree sector localization
- Polygon localization framework
- MEC based clear decision
- Adaptive third measurement
- Target queue
- Nearest neighbor + 2-opt route optimization
- Statistics logging

Build:
mkdir build
cd build
cmake ..
make

Need:
include/json.hpp

Modify ROBOT_ID in api.cpp.
