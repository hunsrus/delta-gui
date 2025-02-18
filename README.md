# delta-gui

## Installation

### Debian

Install dependencies

```
apt install git build-essential cmake xorg libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libglu1-mesa-dev
```

Clone this repository

```
git clone https://github.com/hunsrus/delta-gui.git
```

Make a build folder inside

```
cd delta-gui && mkdir build && cd build
```

Run CMake and generate build files

```
cmake ..
```

Build the project

```
make
```

You can then run the program named ```example```.
