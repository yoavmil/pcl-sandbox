cmake_minimum_required (VERSION 3.10)
project(PclSandbox)

find_package(PCL CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)

include("${VTK_USE_FILE}")

add_executable(prism 
    prism.cpp 
)

target_link_libraries(prism PRIVATE ${PCL_LIBRARIES})
target_link_libraries(prism PRIVATE fmt::fmt-header-only) 

