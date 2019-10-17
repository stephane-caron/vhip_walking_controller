# Copyright (c) 2018-2019, CNRS-UM LIRMM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
    gui/Controller.cpp)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/vhip_walking/utils/stats.h)

set(CONTROLLER_NAME "${PROJET_NAME}_controller")

add_library(${PROJECT_NAME} SHARED ${CONTROLLER_SRC} ${CONTROLLER_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS")
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} copra::copra eigen-lssol::eigen-lssol mc_rtc::mc_control_fsm)
install(TARGETS ${PROJECT_NAME} DESTINATION ${MC_RTC_LIBDIR}/mc_controller)

add_library(${CONTROLLER_NAME} SHARED lib.cpp)
set_target_properties(${CONTROLLER_NAME}
  PROPERTIES
    PREFIX ""
    OUTPUT_NAME "${PROJECT_NAME}"
    LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}")
target_link_libraries(${CONTROLLER_NAME} ${PROJECT_NAME})
install(TARGETS ${CONTROLLER_NAME} DESTINATION ${MC_RTC_LIBDIR}/mc_controller)

set(CONF_OUT "$ENV{HOME}/.config/mc_rtc/controllers/VHIPWalking.conf")
set(AROBASE "@")
set(VHIP_WALKING_STATES_DIR "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/states/")
set(VHIP_WALKING_STATES_DATA_DIR "${vhip_walkING_STATES_DIR}/data")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/../etc/VHIPWalking.conf.cmake" "${CONF_OUT}")
unset(AROBASE)

set(VHIP_WALKING_STATES_DIR "${MC_RTC_LIBDIR}/mc_controller/${PROJECT_NAME}/states")
set(VHIP_WALKING_STATES_DATA_DIR "${vhip_walkING_STATES_DIR}/data")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/../etc/VHIPWalking.conf.cmake" "${CMAKE_CURRENT_BINARY_DIR}/etc/VHIPWalking.conf")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/VHIPWalking.conf"
    DESTINATION "${MC_RTC_LIBDIR}/mc_controller/etc/")

add_subdirectory(states)
