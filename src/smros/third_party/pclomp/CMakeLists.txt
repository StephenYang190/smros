file(GLOB NDT_OMP_FILES *.cpp)
add_library(ndt_omp SHARED
        ${NDT_OMP_FILES})