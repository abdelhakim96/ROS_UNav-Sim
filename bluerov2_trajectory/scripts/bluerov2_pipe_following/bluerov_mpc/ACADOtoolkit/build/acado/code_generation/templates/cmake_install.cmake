# Install script for directory: /home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/acado/code_generation/templates" TYPE FILE FILES
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/acado_EH_solver_mex.c.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/acado_auxiliary_functions.c.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/acado_auxiliary_functions.h.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/acado_auxiliary_sim_functions.c.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/acado_auxiliary_sim_functions.h.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/acado_common_header.h.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/acado_forces_generator.m.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/acado_forces_generator.py.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/acado_hessian_regularization.c.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/acado_solver_mex.c.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/acado_solver_sfunction.c.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/acado_solver_sfunction.h.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/dummy_test_file.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/forces_interface.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/hpmpc_interface.c.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/integrator_mex.c.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/make_acado_EH_solver_qpdunes.m.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/make_acado_EH_solver_qpoases.m.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/make_acado_EH_solver_qpoases3.m.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/make_acado_block_solver_qpdunes.m.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/make_acado_integrator.m.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/make_acado_solver_forces.m.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/make_acado_solver_hpmpc.m.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/make_acado_solver_qpdunes.m.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/make_acado_solver_qpoases.m.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/make_acado_solver_qpoases3.m.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/make_acado_solver_sfunction.m.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/makefile.EH_qpdunes.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/makefile.EH_qpoases.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/makefile.EH_qpoases3.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/makefile.forces.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/makefile.hpmpc.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/makefile.integrator.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/makefile.qpdunes.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/makefile.qpoases.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/makefile.qpoases3.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/qpdunes_interface.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/qpdunes_split_interface.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/qpoases3_interface.c.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/qpoases3_interface.h.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/qpoases_interface.cpp.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/qpoases_interface.hpp.in"
    "/home/hakim/Desktop/ACADOtoolkit/acado/code_generation/templates/templates.hpp.in"
    "/home/hakim/Desktop/ACADOtoolkit/build/acado/code_generation/templates/templates.hpp"
    )
endif()

