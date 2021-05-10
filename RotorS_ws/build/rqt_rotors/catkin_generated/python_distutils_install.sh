#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/src/rqt_rotors"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/install/lib/python2.7/dist-packages:/home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/build" \
    "/usr/bin/python2" \
    "/home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/src/rqt_rotors/setup.py" \
     \
    build --build-base "/home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/build/rqt_rotors" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/install" --install-scripts="/home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/install/bin"