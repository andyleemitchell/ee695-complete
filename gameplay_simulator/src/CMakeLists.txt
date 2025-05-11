# libraries go here

add_library(game_lib
    game.cpp player.cpp
)

target_include_directories(game_lib
    PUBLIC ../include
)
