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

echo_and_run cd "/home/racecar/team-ws/src/racecar/monte_carlo_localization"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/racecar/team-ws/install/lib/python2.7/dist-packages:/home/racecar/team-ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/racecar/team-ws/build" \
    "/usr/bin/python" \
    "/home/racecar/team-ws/src/racecar/monte_carlo_localization/setup.py" \
    build --build-base "/home/racecar/team-ws/build/racecar/monte_carlo_localization" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/racecar/team-ws/install" --install-scripts="/home/racecar/team-ws/install/bin"
