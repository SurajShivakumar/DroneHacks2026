# Source after /opt/ros/jazzy/setup.bash: overlays the privately unpacked Isaac ROS 4.0
# (sim/organizer/fetch_isaac_ros.py) so isaac_ros_visual_slam & co. resolve like an apt install.
_ROOT="${ISAAC_ROS_PREFIX:-${ORGANIZER_SIM:-$HOME/organizer-sim}/isaac_ros}/root"
_IR="$_ROOT/opt/ros/jazzy"
export AMENT_PREFIX_PATH="$_IR:${AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$_IR:${CMAKE_PREFIX_PATH}"
_ARCH="$(uname -m)-linux-gnu"
_LIBS="$_IR/lib:$_IR/lib/$_ARCH:$_ROOT/opt/nvidia/vpi4/lib/$_ARCH"
# CUDA 13 runtime (cudart/cublas/cusolver) when not installed system-wide, e.g. from pip
# wheels: python3 -m venv cuda-venv && cuda-venv/bin/pip install nvidia-cuda-runtime==13.* nvidia-cublas==13.* nvidia-cusolver==12.*
[ -n "$CUDA13_LIB" ] && _LIBS="$_LIBS:$CUDA13_LIB"
# GXF extension libraries live in share/<pkg>/gxf/lib/<ext>/
for _d in $(find "$_IR/share" -path "*/gxf/lib*" -name "*.so" -printf "%h\n" 2>/dev/null | sort -u); do
    _LIBS="$_LIBS:$_d"
done
export LD_LIBRARY_PATH="$_LIBS:${LD_LIBRARY_PATH}"
for _d in "$_IR"/lib/python3*/site-packages; do export PYTHONPATH="$_d:${PYTHONPATH}"; done
export PATH="$_IR/bin:${PATH}"
unset _IR _ROOT _LIBS _d _ARCH
