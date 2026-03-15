cmake_minimum_required(VERSION 4.0)
project(C_SoftwareRenderer C)

set(CMAKE_C_STANDARD 11)

set(SDL3_DIR "C:/c_libs/SDL3-3.4.0/cmake")
find_package(SDL3 CONFIG REQUIRED)

add_executable(C_SoftwareRenderer src/main/main.c
        src/main/window.c
        src/main/mesh.c
        src/main/vector.c
)

target_link_libraries(C_SoftwareRenderer PRIVATE SDL3::SDL3)