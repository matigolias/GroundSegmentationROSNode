inherit ros_distro_melodic

DESCRIPTION = "alfa_pd"
SECTION = "devel"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://package.xml;;beginline=16;endline=16;md5=2feaf30a620f46f06a4b016624acf46f"

ROS_CN = "alfa-pd"
ROS_BPN = "alfa-pd"

ROS_BUILD_DEPENDS = " \
    dynamic-reconfigure \
    geometry-msgs \
    message-filters \
    nodelet \
    nodelet-topic-tools \
    pcl-conversions \
    pcl-msgs \
    pluginlib \
    rosbag \
    rosconsole \
    roscpp \
    roslib \
    sensor-msgs \
    std-msgs \
    message-generation \
    alfa-msg \
"

ROS_EXEC_DEPENDS = " \
    dynamic-reconfigure \
    geometry-msgs \
    message-filters \
    nodelet \
    nodelet-topic-tools \
    pcl-conversions \
    pcl-msgs \
    pluginlib \
    rosbag \
    rosconsole \
    roscpp \
    roslib \
    sensor-msgs \
    std-msgs \
    message-generation \   
"

ROS_BUILDTOOL_DEPENDS = " \
    catkin-native \
"

ROS_EXPORT_DEPENDS = " \
    dynamic-reconfigure \
    geometry-msgs \
    message-filters \
    nodelet \
    nodelet-topic-tools \
    pcl-conversions \
    pcl-msgs \
    pluginlib \
    rosbag \
    rosconsole \
    roscpp \
    roslib \
    sensor-msgs \
    std-msgs \
    message-generation \
"
# ROS_BUILDTOOL_EXPORT_DEPENDS = ""
# ROS_TEST_DEPENDS = ""
DEPENDS = "${ROS_BUILD_DEPENDS} ${ROS_BUILDTOOL_DEPENDS}"
# DEPENDS = "roscpp catkin rospy std-msgs"
# RDEPENDS = "roscpp rospy std-msgs"
RDEPENDS_${PN} += "${ROS_EXEC_DEPENDS}"
DEPENDS += "${ROS_EXPORT_DEPENDS}"
# SRC_URI = "git://github.com/vmayoral/beginner_tutorials.git"
SRC_URI = "git://github.com/DrChipMaster/alfa_pd_node.git"
#SRC_URI = "file:://alfa_pd_node.tar.gz"

SRCREV = "${AUTOREV}"
PV = "1.1.3+gitr${SRCPV}"

S = "${WORKDIR}/git"

ROS_BUILD_TYPE = "catkin"

inherit ros_${ROS_BUILD_TYPE}
