cmake_minimum_required(VERSION 3.5.0)
project(tesseract_serialization VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(tesseract_common REQUIRED)


# Generate Protobuf library: ${PROJECT_NAME}_proto
find_package(Protobuf REQUIRED)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HEADERS proto/addressbook.proto)



add_library(${PROJECT_NAME}_protobuf SHARED ${PROTO_SRCS})
target_link_libraries(${PROJECT_NAME}_protobuf PUBLIC ${PROTOBUF_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_protobuf PUBLIC)
target_include_directories(${PROJECT_NAME}_protobuf PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_protobuf SYSTEM PUBLIC
    ${PROTOBUF_INCLUDE_DIRS}
    )


#Executable setup
set(executable_name proto_example1)
add_executable(${executable_name} src/add_person.cpp)
target_link_libraries(${executable_name} ${PROJECT_NAME}_protobuf)
target_include_directories(${executable_name} SYSTEM PUBLIC ${tesseract_serialization_protobuf_INCLUDE_DIRS})

set(executable_name proto_example2)
add_executable(${executable_name} src/list_people.cpp)
target_link_libraries(${executable_name} ${PROJECT_NAME}_protobuf)
target_include_directories(${executable_name} SYSTEM PUBLIC ${tesseract_serialization_protobuf_INCLUDE_DIRS})


tesseract_configure_package(${PROJECT_NAME}_protobuf proto_example1 proto_example2)

# Mark cpp header files for installation
install(FILES ${PROTO_HEADERS}
  DESTINATION include/${PROJECT_NAME}
)

#install(FILES
#  "${CMAKE_CURRENT_LIST_DIR}/cmake/tesseract_macros.cmake"
#  DESTINATION lib/cmake/${PROJECT_NAME})

#if (TESSERACT_ENABLE_TESTING)
#  enable_testing()
#  tesseract_add_run_tests_target()
#  add_subdirectory(test)
#endif()
