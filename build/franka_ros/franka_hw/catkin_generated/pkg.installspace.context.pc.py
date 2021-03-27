# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/home/crslab/libfranka/include".split(';') if "${prefix}/include;/home/crslab/libfranka/include" != "" else []
PROJECT_CATKIN_DEPENDS = "controller_interface;hardware_interface;joint_limits_interface;roscpp;urdf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lfranka_hw;/home/crslab/libfranka/build/libfranka.so.0.7.1".split(';') if "-lfranka_hw;/home/crslab/libfranka/build/libfranka.so.0.7.1" != "" else []
PROJECT_NAME = "franka_hw"
PROJECT_SPACE_DIR = "/home/crslab/catkin_main/install"
PROJECT_VERSION = "0.7.0"
