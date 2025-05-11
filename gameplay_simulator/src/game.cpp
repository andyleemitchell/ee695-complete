#include "c4sim/game.h"

namespace c4sim
{
    
    Game::Game(std::unique_ptr<Player> p1, std::unique_ptr<Player> p2) :
        player1(std::move(p1)), player2(std::move(p2)) {
            gameState.board_state.fill('-');
    }

    GameStatus Game::game_tick() {
        // check for wins or draws
        if (is_player1_winner()) {
            gameStatus = GameStatus::PLAYER1_WIN;
            return gameStatus;
        }
        if (is_player2_winner()) {
            gameStatus = GameStatus::PLAYER2_WIN;
            return gameStatus;
        }
        if (is_draw()) {
            gameStatus = GameStatus::DRAW;
            return gameStatus;
        }

        // make a move if the game is still on
        std::uint8_t move = -1;
        if (is_player1_move()) {
            move = player1->choose_move(gameState, get_valid_moves(), num_moves);
        } else {
            move = player2->choose_move(gameState, get_valid_moves(), num_moves);
        }
        if (move < WIDTH)
            make_move(move);
        return gameStatus;
    }

    bool Game::is_win(std::uint64_t bitboard) {
        const std::array<std::uint8_t, 4> directions {1, 7, 6, 8};
        uint64_t temp_bb;
        for (auto &direction : directions) {
            temp_bb = bitboard & (bitboard >> direction);
            if ((temp_bb & (temp_bb >> 2 * direction)) != 0) return true;
        }
        return false;
    }

    void Game::make_move(std::uint8_t move) {
        if (num_moves >= MAX_MOVES)
            return;
        std::uint64_t one = 1;
        std::uint64_t temp_move = one << gameState.heights[move]++;
        if (is_player1_move()) {
            gameState.red ^= temp_move;
        } else {
            gameState.yellow ^= temp_move;
        }
        moves_made[num_moves++] = move;
    }

    void Game::undo_last_move() {
        std::uint64_t one = 1;
        std::uint8_t column = moves_made[--num_moves];
        std::uint64_t move = one << --gameState.heights[column];
        if (is_player1_move()) {
            gameState.red ^= move;
        } else {
            gameState.yellow ^= move;
        }
    }

    std::vector<std::uint8_t> Game::get_valid_moves() {
        std::uint64_t one = 1;
        std::vector<std::uint8_t> moves {};
        std::uint64_t TOP = 0b1000000'1000000'1000000'1000000'1000000'1000000'1000000;
        for (std::uint8_t col = 0; col < WIDTH; col++) {
            if ((TOP & (one << gameState.heights[col])) == 0) moves.push_back(col);
        }
        return moves;
    }

    bool Game::is_move_valid(std::uint8_t move) {
        auto valid_moves_temp = get_valid_moves();
        if (std::count(valid_moves_temp.begin(), valid_moves_temp.end(), move)) {
            return true;
        }
        else {
            return false;
        }
    }

    bool Game::is_player1_winner() {
        return is_win(gameState.red);
    }

    bool Game::is_player2_winner() {
        return is_win(gameState.yellow);
    }

    bool Game::is_draw() {
        return num_moves >= MAX_MOVES;
    }

    void Game::generate_state(std::uint64_t bitboard, char symbol) {
        std::uint64_t one = 1;
        for (int i = 0; i < 64; ++i) {
            if ((one << i) & bitboard)
            gameState.board_state[i] = symbol;
        }
    }
    
    std::array<char, 64> Game::get_board() {
        generate_state(gameState.red, 'X');
        generate_state(gameState.yellow, 'O');

        return gameState.board_state;
    }

    std::vector<std::uint8_t> Game::get_moves_made() {
        std::vector<std::uint8_t> temp {};
        for (auto i = 0; i < num_moves ; ++i) {
            temp.push_back(moves_made[i]);
        }
        return temp;
    }

} // namespace c4sim

