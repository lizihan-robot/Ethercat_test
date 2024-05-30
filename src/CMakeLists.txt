cmake_minimum_required(VERSION 3.10)  
 
project(TestEthercat)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
find_package(Threads REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Debug)     
 
 
include_directories(
    /home/lizihan/ethercat_test/lib/SOEM/install/include
)
 
link_directories(
    /home/lizihan/ethercat_test/lib/SOEM/install/lib
)
add_executable(main.o main.cpp) 

target_link_libraries(main.o soem Threads::Threads)
set_target_properties(main.o PROPERTIES INSTALL_RPATH "/home/lizihan/ethercat_test/lib/SOEM/install/include")