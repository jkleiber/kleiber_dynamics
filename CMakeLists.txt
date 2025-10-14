
add_subdirectory(quadcopter)

add_library(kleiber-dynamics INTERFACE)
target_link_libraries(kleiber-dynamics
    INTERFACE
    kleiber-dynamics-quadcopter
)