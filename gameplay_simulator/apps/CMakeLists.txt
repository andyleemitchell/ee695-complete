set(SOURCES
    c4sim.cpp
)

add_executable(c4sim 
    ${SOURCES}
)

target_include_directories(c4sim
    PUBLIC ../include
)

target_link_libraries(c4sim
    PUBLIC game_lib
)


add_executable(c4sim_gui
    c4sim_gui.cpp
)
target_include_directories(c4sim_gui PUBLIC ../include)
target_link_libraries(c4sim_gui PUBLIC raylib raylib_cpp game_lib)


add_executable(test_routine
    test_routine.cpp
)
target_include_directories(test_routine PUBLIC ../include)
target_link_libraries(test_routine PUBLIC raylib raylib_cpp game_lib)
