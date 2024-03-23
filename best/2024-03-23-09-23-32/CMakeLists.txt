project(CodeCraftSDK)
cmake_minimum_required (VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

if (!WIN32)
    link_libraries(pthread rt m)
endif (!WIN32)


# 设置编译选项
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -ftree-vectorize -funroll-loops")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -Rpass=loop-vectorize")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 /arch:AVX2")
endif()

# AUX_SOURCE_DIRECTORY(. src)
file(GLOB SRC_FILES "*.cpp")
ADD_EXECUTABLE(main ${SRC_FILES})

# target_compile_definitions(main PRIVATE NDEBUG)  # 设置预定义宏MY_DEFINE