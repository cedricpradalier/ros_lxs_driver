project(lxs36)
cmake_minimum_required(VERSION 2.4.6)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)




set(PROJECT_NAME lxs36)

find_package(Boost REQUIRED system)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_definitions(-g -ggdb -Wall -O3 -pthread -std=c++11 )

# include all libs so far
include_directories(
    include
)

add_library(lxs_lib
    src/LxS_Lib/LxS_Config.cpp
    # src/LxS_Lib/LxS_Device.cpp
    src/LxS_Lib/LxS_Param.cpp
    src/LxS_Lib/LxS_ParamSet.cpp
)
target_link_libraries(lxs_lib ${Boost_ASIO_LIBRARY} ${Boost_SYSTEM_LIBRARY}
    pthread)


add_executable(udp_server src/udp_server.cpp)
target_link_libraries(udp_server ${Boost_ASIO_LIBRARY} ${Boost_SYSTEM_LIBRARY} pthread)


add_executable(udp_client src/udp_client.cpp)
target_link_libraries(udp_client ${Boost_ASIO_LIBRARY} ${Boost_SYSTEM_LIBRARY} pthread)

add_executable(test_lxs src/test_lxs.cpp)
target_link_libraries(test_lxs ${Boost_ASIO_LIBRARY} ${Boost_SYSTEM_LIBRARY} pthread)



