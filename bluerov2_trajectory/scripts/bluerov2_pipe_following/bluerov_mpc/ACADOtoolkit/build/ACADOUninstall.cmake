################################################################################
#
# Description:
#	CMake script for uninstalling ACADO
#
# Author:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2011.
#
# Usage:
#	- /
#
# TODO:
#	- Add code for deleting ACADO folders
#
################################################################################

IF( NOT EXISTS "/home/hakim/Desktop/ACADOtoolkit/build/install_manifest.txt" )
    MESSAGE( FATAL_ERROR "Cannot find install manifest: \"/home/hakim/Desktop/ACADOtoolkit/build/install_manifest.txt\"" )
ENDIF( NOT EXISTS "/home/hakim/Desktop/ACADOtoolkit/build/install_manifest.txt" )

FILE( READ "/home/hakim/Desktop/ACADOtoolkit/build/install_manifest.txt" files )
STRING( REGEX REPLACE "\n" ";" files "${files}" )
CMAKE_POLICY(SET CMP0007 NEW)
LIST( REVERSE files )
CMAKE_POLICY(SET CMP0007 OLD)
FOREACH( file ${files} )
    MESSAGE(  STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"" )
    IF( EXISTS "$ENV{DESTDIR}${file}" )
        EXECUTE_PROCESS(
            COMMAND /usr/bin/cmake -E remove "$ENV{DESTDIR}${file}"
            OUTPUT_VARIABLE rm_out
            RESULT_VARIABLE rm_retval
        )
        IF( NOT ${rm_retval} EQUAL 0 )
            MESSAGE( FATAL_ERROR "Problem when removing \"$ENV{DESTDIR}${file}\"" )
        ENDIF( NOT ${rm_retval} EQUAL 0 )
    ELSE( EXISTS "$ENV{DESTDIR}${file}" )
        MESSAGE( STATUS "File \"$ENV{DESTDIR}${file}\" does not exist." )
    ENDIF( EXISTS "$ENV{DESTDIR}${file}" )
ENDFOREACH( file )

