#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "highs" for configuration "Release"
set_property(TARGET highs APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(highs PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/highs.exe"
  )

list(APPEND _IMPORT_CHECK_TARGETS highs )
list(APPEND _IMPORT_CHECK_FILES_FOR_highs "${_IMPORT_PREFIX}/bin/highs.exe" )

# Import target "libhighs" for configuration "Release"
set_property(TARGET libhighs APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(libhighs PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/libhighs.dll.a"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/libhighs.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS libhighs )
list(APPEND _IMPORT_CHECK_FILES_FOR_libhighs "${_IMPORT_PREFIX}/lib/libhighs.dll.a" "${_IMPORT_PREFIX}/bin/libhighs.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
