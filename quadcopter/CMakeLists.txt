
file(GLOB SOURCE_LIST
    CONFIGURE_DEPENDS
    "src/*.cpp")
file(GLOB HEADER_LIST
    CONFIGURE_DEPENDS
    "include/quadcopter/*.h")

add_library(kleiber-dynamics-quadcopter
    STATIC
    ${SOURCE_LIST}
    ${HEADER_LIST})

target_include_directories(kleiber-dynamics-quadcopter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kleiber-dynamics-quadcopter PRIVATE Eigen3::Eigen)