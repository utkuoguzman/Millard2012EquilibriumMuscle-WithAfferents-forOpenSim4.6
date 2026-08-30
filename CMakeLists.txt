###########################################
## For building with the rest of OpenSim ##
###########################################

cmake_minimum_required(VERSION 3.14)

# Define project
project(osimMillard12EqWithAff LANGUAGES CXX)

find_package(OpenSim REQUIRED)

file(GLOB SOURCE_FILES *.h *.cpp)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenSim_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenSim_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "osimMillard12EqWithAff")