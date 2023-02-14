################################################################################
#
# Description:
#	ACADO Toolkit package configuration file
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2011 - 2014.
#
# NOTE:
#	- Linux/Unix only.
#
# Usage:
#	- Linux/OSX:
#		* Users are supposed to source this file into ~/.bashrc (or ~/.profile).
#
################################################################################

################################################################################
#
# Definitions for both users and developers.
#
################################################################################

# 
# Tell the user project where to find our headers, libraries and external
# packages, etc.
#
export ACADO_ENV_INCLUDE_DIRS="/home/hakim/Desktop/ACADOtoolkit;/home/hakim/Desktop/ACADOtoolkit/acado;/home/hakim/Desktop/ACADOtoolkit/external_packages;/home/hakim/Desktop/ACADOtoolkit/external_packages/qpOASES-3.2.0/include;/home/hakim/Desktop/ACADOtoolkit/build"
export ACADO_ENV_LIBRARY_DIRS="/home/hakim/Desktop/ACADOtoolkit/build/lib"
export ACADO_ENV_EXTERNAL_PACKAGES_DIR="/home/hakim/Desktop/ACADOtoolkit/external_packages"
export ACADO_ENV_CMAKE_DIR="/home/hakim/Desktop/ACADOtoolkit/cmake"
 
#
# List of ACADO static libraries
#
export ACADO_ENV_STATIC_LIBRARIES=""
#
# List of ACADO shared libraries
#
export ACADO_ENV_SHARED_LIBRARIES="acado_toolkit_s"

#
# ACADO is shipped with embedded version of qpOASES. Here is specified
# where source and header files reside
#
export ACADO_ENV_QPOASES_EMBEDDED_SOURCES="/home/hakim/Desktop/ACADOtoolkit/external_packages/qpoases/SRC/Bounds.cpp;/home/hakim/Desktop/ACADOtoolkit/external_packages/qpoases/SRC/CyclingManager.cpp;/home/hakim/Desktop/ACADOtoolkit/external_packages/qpoases/SRC/MessageHandling.cpp;/home/hakim/Desktop/ACADOtoolkit/external_packages/qpoases/SRC/QProblem.cpp;/home/hakim/Desktop/ACADOtoolkit/external_packages/qpoases/SRC/Utils.cpp;/home/hakim/Desktop/ACADOtoolkit/external_packages/qpoases/SRC/Constraints.cpp;/home/hakim/Desktop/ACADOtoolkit/external_packages/qpoases/SRC/Indexlist.cpp;/home/hakim/Desktop/ACADOtoolkit/external_packages/qpoases/SRC/QProblemB.cpp;/home/hakim/Desktop/ACADOtoolkit/external_packages/qpoases/SRC/SubjectTo.cpp;/home/hakim/Desktop/ACADOtoolkit/external_packages/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp"
export ACADO_ENV_QPOASES_EMBEDDED_INC_DIRS="/home/hakim/Desktop/ACADOtoolkit/external_packages/qpoases/;/home/hakim/Desktop/ACADOtoolkit/external_packages/qpoases/INCLUDE;/home/hakim/Desktop/ACADOtoolkit/external_packages/qpoases/SRC"

