# - Check for the presence of Freespace
#
# The following variables are set when Freespace is found:
#  HAVE_Freespace       = Set to true, if all components of Freespace
#                          have been found.
#  Freespace_INCLUDES   = Include path for the header files of Freespace
#  Freespace_LIBRARIES  = Link these to use Freespace

## -----------------------------------------------------------------------------
## Check for the header files

find_path (Freespace_INCLUDES freespace/freespace.h
  PATHS /usr/local/include /usr/include
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (Freespace_LIBRARIES libfreespace.so
  PATHS /usr/local/lib /usr/lib /lib /sw/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (Freespace_INCLUDES AND Freespace_LIBRARIES)
  set (HAVE_Freespace TRUE)
else (Freespace_INCLUDES AND Freespace_LIBRARIES)
  if (NOT Freespace_FIND_QUIETLY)
    if (NOT Freespace_INCLUDES)
      message (STATUS "Unable to find Freespace header files!")
    endif (NOT Freespace_INCLUDES)
    if (NOT Freespace_LIBRARIES)
      message (STATUS "Unable to find Freespace library files!")
    endif (NOT Freespace_LIBRARIES)
  endif (NOT Freespace_FIND_QUIETLY)
endif (Freespace_INCLUDES AND Freespace_LIBRARIES)

if (HAVE_Freespace)
  if (NOT Freespace_FIND_QUIETLY)
    message (STATUS "Found components for Freespace")
    message (STATUS "Freespace_INCLUDES = ${Freespace_INCLUDES}")
    message (STATUS "Freespace_LIBRARIES     = ${Freespace_LIBRARIES}")
  endif (NOT Freespace_FIND_QUIETLY)
else (HAVE_Freespace)
  if (Freespace_FIND_REQUIRED)
    message (FATAL_ERROR "Could not find Freespace!")
  endif (Freespace_FIND_REQUIRED)
endif (HAVE_Freespace)

mark_as_advanced (
  HAVE_Freespace
  Freespace_LIBRARIES
  Freespace_INCLUDES
  )
