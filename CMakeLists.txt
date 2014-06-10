#Change this if you need to target a specific CMake version
cmake_minimum_required(VERSION 2.8)
project(ViAGE)

#Linux C++11
list(APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Enable debug symbols by default
if(CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Include header files in Compiler's path
include_directories("${PROJECT_SOURCE_DIR}/include/")

# Define Executable Source and Headers
set(SOURCE
        ${PROJECT_SOURCE_DIR}/src/conectrobo.cpp
        ${PROJECT_SOURCE_DIR}/src/main.cpp
        ${PROJECT_SOURCE_DIR}/src/mensurium.cpp
        ${PROJECT_SOURCE_DIR}/src/prog.cpp
        ${PROJECT_SOURCE_DIR}/src/pugixml.cpp
)

set(HEADERS
        ${PROJECT_SOURCE_DIR}/include/VIAGE/conectrobo.hpp
        ${PROJECT_SOURCE_DIR}/include/VIAGE/mensurium.hpp
        ${PROJECT_SOURCE_DIR}/include/VIAGE/prog.hpp
        ${PROJECT_SOURCE_DIR}/include/VIAGE/pugiconfig.hpp
        ${PROJECT_SOURCE_DIR}/include/VIAGE/pugixml.hpp
)

# Define Executable
set(EXECUTABLE_NAME ViAGE)
add_executable(${EXECUTABLE_NAME} ${SOURCE} ${HEADERS})

# Find OpenCV libraries
find_package(OpenCV REQUIRED)
target_link_libraries(${EXECUTABLE_NAME} ${OpenCV_LIBS} -L/usr/local/cuda-6.0/lib64 -lcudart -lnppc -lnppi -lnpps -lcufft)

# Find Cuda libs
find_package(CUDA REQUIRED)
target_link_libraries(${EXECUTABLE_NAME} ${CUDA_LIBRARIES})

# Install Target
install(TARGETS ${EXECUTABLE_NAME} DESTINATION .)
file(COPY config DESTINATION .)

# CPack packaging
include(InstallRequiredSystemLibraries)
include(CPack)
