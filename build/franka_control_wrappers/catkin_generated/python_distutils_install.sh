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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/crslab/catkin_main/src/franka_control_wrappers"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/crslab/catkin_main/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/crslab/catkin_main/install/lib/python2.7/dist-packages:/home/crslab/catkin_main/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/crslab/catkin_main/build" \
    "/usr/bin/python2" \
    "/home/crslab/catkin_main/src/franka_control_wrappers/setup.py" \
    build --build-base "/home/crslab/catkin_main/build/franka_control_wrappers" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/crslab/catkin_main/install" --install-scripts="/home/crslab/catkin_main/install/bin"
