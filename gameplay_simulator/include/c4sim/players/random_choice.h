/**
 * This is a player that makes a random move each time. Useful for baselines.
 */

#ifndef RANDOM_CHOICE_H
#define RANDOM_CHOICE_H

#include "c4sim/player.h"
#include <cstdlib>
#include <ctime>

namespace c4sim {

class RandomChoice : public Player {
    private:
        int entries = 0;
        int max_entries = 30;
        std::uint8_t choice;
    public:
        RandomChoice() { std::srand(static_cast<unsigned int>(std::time(nullptr))); }

        std::uint8_t choose_move(const GameState& boardState, 
            const std::vector<std::uint8_t>& valid_moves, std::uint8_t num_moves) override {
                ++entries;
                if (entries < max_entries)
                    return -1;
                int rand_choice = std::rand() % valid_moves.size();
                choice = valid_moves[rand_choice];

                entries = 0;
                return choice;
        }

};

} // namespace c4sim

#endif//RANDOM_CHOICE_H
