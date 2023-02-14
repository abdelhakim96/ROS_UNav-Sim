################################################################################
#
# Description:
#	ACADO Toolkit package configuration file
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2011 - 2013.
#
# NOTE:
#	- /
#
# Usage:
#	- /
#
################################################################################

################################################################################
#
# Configurable section
#
################################################################################

# 
# Tell the user project where to find our headers, libraries and external
# packages, etc.
#
SET( ACADO_INCLUDE_DIRS "/usr/local/include/acado" )
SET( ACADO_LIBRARY_DIRS "/usr/local/lib" )
SET( ACADO_EXTERNAL_PACKAGES_DIR /usr/local/share/acado/external_packages )
SET( ACADO_CMAKE_DIR /usr/local/share/acado/cmake )
 
#
# List of ACADO static libraries
#
SET( ACADO_STATIC_LIBRARIES  )
#
# List of ACADO shared libraries
#
SET( ACADO_SHARED_LIBRARIES acado_toolkit_s )

#
# ACADO is shipped with embedded version of qpOASES. Here is specified
# where source and header files reside
#
SET( ACADO_QPOASES_EMBEDDED_SOURCES /usr/local/share/acado/external_packages/qpoases/SRC/Bounds.cpp;/usr/local/share/acado/external_packages/qpoases/SRC/CyclingManager.cpp;/usr/local/share/acado/external_packages/qpoases/SRC/MessageHandling.cpp;/usr/local/share/acado/external_packages/qpoases/SRC/QProblem.cpp;/usr/local/share/acado/external_packages/qpoases/SRC/Utils.cpp;/usr/local/share/acado/external_packages/qpoases/SRC/Constraints.cpp;/usr/local/share/acado/external_packages/qpoases/SRC/Indexlist.cpp;/usr/local/share/acado/external_packages/qpoases/SRC/QProblemB.cpp;/usr/local/share/acado/external_packages/qpoases/SRC/SubjectTo.cpp;/usr/local/share/acado/external_packages/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp )
SET( ACADO_QPOASES_EMBEDDED_INC_DIRS /usr/local/share/acado/external_packages/qpoases/;/usr/local/share/acado/external_packages/qpoases/INCLUDE;/usr/local/share/acado/external_packages/qpoases/SRC )

################################################################################
#
# Validation
#
################################################################################

# TODO

