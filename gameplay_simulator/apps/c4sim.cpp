#include "c4sim/game.h"
#include "c4sim/player.h"
#include "c4sim/players/terminal_user.h"
#include "c4sim/players/ab_minimax.h"
#include "c4sim/players/random_choice.h"
#include "c4sim/players/mcts.h"

#include <iostream>

void print_board_state(std::array<char, 64> board_state);

int main(void) {

    std::unique_ptr<c4sim::Player> p1 = std::make_unique<c4sim::MonteCarloTreeSearch>();
    std::unique_ptr<c4sim::Player> p2 = std::make_unique<c4sim::TerminalHuman>();

    c4sim::Game game(std::move(p1), std::move(p2));

    while (true) {
        print_board_state(game.get_board());
        auto status = game.game_tick();

        if (status == c4sim::GameStatus::IN_PROGRESS) {
            continue;
        } else if (status == c4sim::GameStatus::PLAYER1_WIN) {
            std::cout << "Player 1 wins.\n";
            print_board_state(game.get_board());
            break;
        } else if (status == c4sim::GameStatus::PLAYER2_WIN) {
            std::cout << "Player 2 wins.\n";
            print_board_state(game.get_board());
            break;
        } else if (status == c4sim::GameStatus::DRAW) {
            std::cout << "Game is a draw.\n";
            print_board_state(game.get_board());
            break;
        }
    }

}

void print_board_state(std::array<char, 64> board_state) {
    for (int i = 0; i < 7; ++i) {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    for (auto row = 5; row >= 0; --row) {
        for (auto column = 0; column <= 6; ++column) {
            // std::cout << board_state[row + ((column+1) * 7)];
            std::cout << board_state[row + ((column) * 7)] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}
