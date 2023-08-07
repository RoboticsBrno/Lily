cmake_minimum_required(VERSION 3.0)

set(BOOST_INCLUDE_LIBRARIES thread filesystem system program_options asio date_time)
set(BOOST_ENABLE_CMAKE ON)

include(FetchContent)
include(CTest)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.2.0
)
FetchContent_MakeAvailable(Catch2)


FetchContent_Declare(
  Boost
  GIT_REPOSITORY https://github.com/boostorg/boost.git
  GIT_TAG boost-1.82.0
)
FetchContent_MakeAvailable(Boost)


add_executable(main main.cpp)
target_link_libraries(main PUBLIC visualizer Boost::asio)

add_executable(cobs "cobs.cpp")
target_link_libraries(cobs PUBLIC visualizer Catch2::Catch2WithMain)

add_test(NAME cobs COMMAND ${EXECUTABLE_OUTPUT_PATH}/cobs)
