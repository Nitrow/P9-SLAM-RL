project(ign_py)

cmake_minimum_required(VERSION 3.16)

## Both Boost.python and Python libs are required.
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 3.8.5 REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ign_py
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)

###########
## Build ##
###########




# include Boost and Python.
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)
find_package(ignition-msgs6 QUIET REQUIRED)
find_package(ignition-transport9 QUIET REQUIRED)


set(LIBRARY_TARGET ${PROJECT_NAME})
add_library(${LIBRARY_TARGET} SHARED src/${LIBRARY_TARGET}.cpp)

target_link_libraries(${LIBRARY_TARGET} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ignition-msgs6 ignition-transport9)
target_include_directories(${LIBRARY_TARGET} PRIVATE ${Boost_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})
set_target_properties(${LIBRARY_TARGET} PROPERTIES PREFIX "")
install(TARGETS ${LIBRARY_TARGET} DESTINATION lib/python3/dist-packages/test)

