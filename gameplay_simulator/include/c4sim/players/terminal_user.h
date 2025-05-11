/**
 * This is for a basic terminal player that takes input from stdin.
 */

#ifndef TERMINAL_USER_H
#define TERMINAL_USER_H

#include "c4sim/player.h"
#include <iostream>
#include <algorithm>

namespace c4sim {

class TerminalHuman : public Player {
    private:
        int choice;
    public:
    std::uint8_t choose_move(const GameState& boardState, 
        const std::vector<std::uint8_t>& valid_moves, std::uint8_t num_moves) override {
            prompt_user(valid_moves);
            return choice;
    }

    void prompt_user(const std::vector<std::uint8_t>& valid_moves) {
        while (true)
        {
            std::cout << "Enter column from ("; 
            for (auto &move : valid_moves)
                std::cout << static_cast<int>(move) << ", ";
            std::cout << "): ";
            std::cin  >> choice;
            
            if (std::count(valid_moves.begin(), valid_moves.end(), choice))
                break;
            else
                std::cout << "Not a valid input.\n";
        }
    }
};

} // namespace c4sim

#endif//TERMINAL_USER_H
