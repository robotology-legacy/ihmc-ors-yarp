#.rst:
# FindAsio
# -----------
#
# Find the Asio library.
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following :prop_tgt:`IMPORTED` targets if
# Asio has been found::
#
#   Asio::Asio
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module defines the following variables::
#
#   Asio_FOUND                - System has Asio
#   Asio_INCLUDE_DIRS         - Include directories for Asio
#   Asio_LIBRARIES            - imported targets to link against Asio
#
# Readed enviromental variables
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#
# This module reads hints about search locations from variables::
#
#   Asio_ROOT                 - Directory containing the include and lib directories

#=============================================================================
# Copyright 2017  iCub Facility, Istituto Italiano di Tecnologia
#   Authors: Silvio Traversaro <silvio.traversaro@iit.it>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of YCM, substitute the full
#  License text for the above reference.)

find_path(Asio_INCLUDE_DIR matio.h
          HINTS $ENV{Asio_ROOT})

mark_as_advanced(Asio_INCLUDE_DIR)

if(Asio_INCLUDE_DIR AND NOT TARGET Asio::Asio)
  add_library(Asio::Asio UNKNOWN IMPORTED)
  set_target_properties(Asio::Asio PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${MATIO_INCLUDE_DIR}")

  set(MATIO_LIBRARIES MATIO::MATIO)
  set(MATIO_INCLUDE_DIRS "${MATIO_INCLUDE_DIR}")
endif()


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MATIO
                                  FOUND_VAR MATIO_FOUND
                                  REQUIRED_VARS MATIO_LIBRARIES MATIO_INCLUDE_DIRS)

# Set package properties if FeatureSummary was included
if(COMMAND set_package_properties)
  set_package_properties(MATIO PROPERTIES DESCRIPTION "MATLAB MAT File I/O Library"
                                          URL "http://matio.sf.net")
endif()
