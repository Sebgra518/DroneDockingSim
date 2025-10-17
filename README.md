This guide will teach you how to install Ardupilot and Gazebo on WSL/Ubuntu

# Ardupilot & Gazebo Installation

### 1. Preperation

`sudo apt update && sudo apt upgrade -y`
`sudo apt install build-essential git python3-pip cmake wget curl gnupg lsb-release`

### 2. Install Ardupilot:

##### Clone:

`git clone https://github.com/ArduPilot/ardupilot.git`
`cd ardupilot`
`git submodule update --init --recursive`
`Tools/environment_install/install-prereqs-ubuntu.sh -y`
`. ~/.profile`

##### Build SITL firmware:

`./waf configure --board sitl`
`./waf copter`

##### Test ArduPilot:

`cd ArduCopter`
`sim_vehicle.py -v ArduCopter --map --console`

# 3. Gazebo Installation

Refer to: https://gazebosim.org/docs/harmonic/install_ubuntu/

Test with: `gz sim -v4 -r shapes.sdf`

# 4. Gazebo ArduPilot Plugin Installation

##### Clone and Make:

`cd ~`
`git clone https://github.com/ArduPilot/ardupilot_gazebo.git`
`cd ardupilot_gazebo`
`mkdir build && cd build`
`export GZ_VERSION=harmonic`
`cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo`
`make -j$(nproc)`

##### Set environment variables permanently:

`echo 'export GZ_VERSION=harmonic' >> ~/.bashrc`
`echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc`
`echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc`
`source ~/.bashrc`

# Troubleshooting

##### Problem: Build stops ~40% on `GstCameraPlugin.cc`

##### Solution: `sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl gstreamer1.0-plugins-ugly`



  🛫 **Commands not working**               `mode GUIDED`,         Commands typed into the wrong terminal (Gazebo   Type them inside the ArduPilot console (`MAV>` prompt).
                                            `arm throttle` did     instead of MAVProxy)                             
                                            nothing                                                                 



################ Starting Gazebo with Ardupilot

\~/DroneDockingSim/run_ardupilot.sh



# HAVE WSL USE DEDICATED GPU

If you're having trouble passing your dedicated GPU to
WLS, type this into Ubuntu Terminal:

`export GALLIUM_DRIVER=d3d12 export MESA_D3D12_DEFAULT_ADAPTER_NAME="<YOUR GPU NAME>" glxinfo -B`

To make it permanent, add this to the bottom of `~/.bashrc`

# WSL GPU selection (keeps you off llvmpipe)

`export GALLIUM_DRIVER=d3d12 export MESA_D3D12_DEFAULT_ADAPTER_NAME="<YOUR GPU NAME>" source "/home/<USER>/DroneDockingSim/ardupilot/Tools/completion/completion.bash"`

# git commands

`cd /` `

`git add .`

`git commit -m "Your descriptive commit message here"`
