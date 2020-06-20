##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_remocon)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

##############################################################################
# Resources
##############################################################################

set(RESOURCES_DIR resources)

add_custom_target(
    resources_${PROJECT_NAME}
    ALL
    COMMAND pyrcc4 -o src/${PROJECT_NAME}/images.py ${RESOURCES_DIR}/images.qrc
    WORKING_DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}
)

##############################################################################
# Installs
##############################################################################

install(
    FILES rqt_remocon_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    PROGRAMS
#        scripts/rocon_remocon_check_up
#        scripts/rocon_remocon_sub
#        scripts/rocon_remocon
        scripts/rqt_remocon
        scripts/wrqt_remocon
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY ui
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

