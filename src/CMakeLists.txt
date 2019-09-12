# Copyright 2018-2019 CNRS-UM LIRMM
#
# This file is part of vhip_walking_controller.
#
# vhip_walking_controller is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of the License,
# or (at your option) any later version.
#
# vhip_walking_controller is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
# General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with vhip_walking_controller. If not, see
# <http://www.gnu.org/licenses/>.

set(CONTROLLER_SRC
    Controller.cpp
    FloatingBaseObserver.cpp
    FootstepPlan.cpp
    HRP4ForceCalibrator.cpp
    ModelPredictiveControl.cpp
    NetWrenchObserver.cpp
    Pendulum.cpp
    Stabilizer.cpp
    SwingFoot.cpp
    gui/Controller.cpp
)

set(CONTROLLER_HDR
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/Contact.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/Controller.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/FloatingBaseObserver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/FootstepPlan.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/HRP4ForceCalibrator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/ModelPredictiveControl.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/NetWrenchObserver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/Pendulum.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/Preview.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/Sole.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/Stabilizer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/State.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/SwingFoot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/utils/LeakyIntegrator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/utils/LowPassVelocityFilter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/utils/clamp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/utils/polynomials.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/utils/rotations.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/utils/stats.h
)

set(FSM_CONTROLLER_LIB mc_control_fsm)
add_library(${PROJECT_NAME} SHARED ${CONTROLLER_SRC} ${CONTROLLER_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS")
pkg_config_use_dependency(${PROJECT_NAME} copra)
pkg_config_use_dependency(${PROJECT_NAME} mc_rtc)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${FSM_CONTROLLER_LIB})
install(TARGETS ${PROJECT_NAME} DESTINATION ${MC_RTC_LIBDIR}/mc_controller)

add_library(${PROJECT_NAME}_controller SHARED lib.cpp)
set_target_properties(${PROJECT_NAME}_controller
  PROPERTIES
    PREFIX ""
    OUTPUT_NAME "${PROJECT_NAME}"
    LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}"
)
target_link_libraries(${PROJECT_NAME}_controller ${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_controller DESTINATION ${MC_RTC_LIBDIR}/mc_controller)

set(CONF_OUT "$ENV{HOME}/.config/mc_rtc/controllers/VHIPWalking.conf")
set(AROBASE "@")
set(VHIP_WALKING_STATES_DIR "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/states/")
set(VHIP_WALKING_STATES_DATA_DIR "${vhip_walkING_STATES_DIR}/data")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/../etc/VHIPWalking.conf.cmake" "${CONF_OUT}")
unset(AROBASE)

set(VHIP_WALKING_STATES_DIR "${MC_RTC_LIBDIR}/mc_controller/vhip_walking_controller/states")
set(VHIP_WALKING_STATES_DATA_DIR "${vhip_walkING_STATES_DIR}/data")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/../etc/VHIPWalking.conf.cmake" "${CMAKE_CURRENT_BINARY_DIR}/etc/VHIPWalking.conf")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/VHIPWalking.conf"
    DESTINATION "${MC_RTC_LIBDIR}/mc_controller/etc/")

add_subdirectory(states)
