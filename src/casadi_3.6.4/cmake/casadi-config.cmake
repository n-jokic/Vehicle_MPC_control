# Config file for the CasADi package
get_filename_component(CASADI_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

include("${CASADI_CMAKE_DIR}/casadi-targets.cmake")

if(OFF AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  target_compile_definitions(casadi INTERFACE _GLIBCXX_USE_CXX11_ABI=0)
endif()
