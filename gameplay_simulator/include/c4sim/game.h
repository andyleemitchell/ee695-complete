#ifndef GAME_H
#define GAME_H

#include <cstdint>
#include <memory>
#include <array>
#include <vector>
#include <algorithm>

#include "c4sim/player.h"

namespace c4sim
{
    // constants
    constexpr int HEIGHT = 6;
    constexpr int WIDTH  = 7;
    constexpr int MAX_MOVES = HEIGHT * WIDTH;

    enum class GameStatus {
        IN_PROGRESS,
        PLAYER1_WIN,
        PLAYER2_WIN,
        DRAW
    };

    struct GameState
    {
        std::uint64_t red {};
        std::uint64_t yellow {};
        std::array<std::uint8_t, WIDTH> heights {0, 7, 14, 21, 28, 35, 42};
        std::array<char, 64> board_state {};
    };

    class Game
    {
    private:
        std::unique_ptr<Player> player1;
        std::unique_ptr<Player> player2;

        std::uint8_t num_moves = 0;
        std::array<std::uint8_t, MAX_MOVES> moves_made = {};

        GameStatus gameStatus = GameStatus::IN_PROGRESS;
        GameState gameState;
        // TODO some of the functions below should be private.
    public:
        Game();
        Game(std::unique_ptr<Player> p1, std::unique_ptr<Player> p2);

        bool is_player1_move() { return not (num_moves & 1L); }
        
        GameStatus game_tick();

        void make_move(std::uint8_t move);
        void undo_last_move();

        std::vector<std::uint8_t> get_valid_moves();
        bool is_move_valid(std::uint8_t move);
        
        bool is_win(std::uint64_t bitboard);
        bool is_player1_winner();
        bool is_player2_winner();
        bool is_draw();

        // for printing
        void generate_state(std::uint64_t bitboard, char symbol);
        std::array<char, 64> get_board();

        std::vector<std::uint8_t> get_moves_made();

    };
    
} // namespace c4sim

#endif//GAME_H
