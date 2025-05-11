/**
 * This is an implementation of the alpha beta pruned minimax algorithm.
 */

#ifndef AB_MINIMAX_H
#define AB_MINIMAX_H

#include "c4sim/player.h"

namespace c4sim {

class AlphaBetaMinimax : public Player {
    private:
        int choice;
    public:
    std::uint8_t choose_move(const GameState& boardState, 
        const std::vector<std::uint8_t>& valid_moves, std::uint8_t num_moves) override {
            return choice;
    }

};

} // namespace c4sim

#endif//AB_MINIMAX_H
