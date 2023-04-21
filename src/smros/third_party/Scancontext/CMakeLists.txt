file(GLOB SCAN_CONTEXT_FILES *.cpp)
add_library(scan_context SHARED
        ${SCAN_CONTEXT_FILES}
        nanoflann.hpp)