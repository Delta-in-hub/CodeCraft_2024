project(CodeCraftSDK)
cmake_minimum_required (VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)


set(CMAKE_CXX_FLAGS "-O3")

if (!WIN32)
    link_libraries(pthread rt m)
endif (!WIN32)

# AUX_SOURCE_DIRECTORY(. src)
file(GLOB SRC_FILES "*.cpp")
ADD_EXECUTABLE(main ${SRC_FILES})

target_compile_definitions(main PRIVATE NDEBUG)  # 设置预定义宏MY_DEFINE