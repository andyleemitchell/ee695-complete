#ifndef PLAYER_H
#define PLAYER_H

#include <cstdint>
#include <vector>


namespace c4sim {
    
    struct GameState;
    /**
     * This is an abstract base class for game players. 
     * 
     * These could be for human players / algorithms / neural nets etc.
     * They need to implement the make move function, which should return an integer corresponding
     * to the column chosen to play (0 indexed, left to right), for the given board state, whose
     * definition can be found in the game.h file.
     */
    class Player {
        public:
        virtual std::uint8_t choose_move(const GameState& boardState, 
            const std::vector<std::uint8_t>& valid_moves, std::uint8_t num_moves) = 0;
        virtual ~Player() {}
    };

    class Human : public Player {
    public:
        std::uint8_t choose_move(const GameState& boardState, const std::vector<std::uint8_t>& valid_moves, std::uint8_t num_moves) override;
    };
    
} // namespace c4sim

#endif//PLAYER_H
