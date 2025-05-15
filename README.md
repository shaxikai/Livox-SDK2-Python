# Livox SDK2 Python

## Brief Introduction
Livox SDK2 Python uses **pybind11** to wrap some of the Livox SDK2 C++ interfaces into Python.

## Environment
Test environment: 
- Ubuntu 
- CMake version 3.16.3
- pybind11 version 2.4.3
- Python 3.8.20

To install pybind11:
```bash
sudo apt install pybind11-dev
```

## Usage
```
mkdir build
cd build
cmake CMAKE_BUILD_TYPE:STRING=Relsease ..
make -j$(nproc)
python ../samples/livox_python/start.py
```

##  Contact
If you have any issue compiling/running Livox SDK2 Python or you would like to know anything about the code, please contact the authors:

    Wei Wang -> shaxikai@outlook.com
