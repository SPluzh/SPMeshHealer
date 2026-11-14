cmake_minimum_required(VERSION 3.16)
project(SPMeshHealer VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

add_library(ufbx STATIC third_party/ufbx/ufbx.c)
target_include_directories(ufbx PUBLIC third_party/ufbx)
if(NOT MSVC)
    target_compile_options(ufbx PRIVATE -O3)
endif()

add_executable(SPMeshHealer
    src/main.cpp
    src/HealerCore.cpp
    src/BinaryIO.cpp
    src/StlIO.cpp
    src/ObjIO.cpp
    src/GozIO.cpp
    src/FbxIO.cpp
)

target_include_directories(SPMeshHealer PRIVATE src third_party)
target_link_libraries(SPMeshHealer PRIVATE CGAL::CGAL Eigen3::Eigen ufbx)

if(OpenMP_CXX_FOUND)
    target_link_libraries(SPMeshHealer PRIVATE OpenMP::OpenMP_CXX)
endif()

if(MINGW)
    target_compile_options(SPMeshHealer PRIVATE -O3 -DNDEBUG -Wa,-mbig-obj)
    target_link_options(SPMeshHealer PRIVATE -static-libgcc -static-libstdc++)
elseif(MSVC)
    target_compile_options(SPMeshHealer PRIVATE /W3 /O2 /utf-8)
else()
    target_compile_options(SPMeshHealer PRIVATE -O3 -DNDEBUG)
endif()
