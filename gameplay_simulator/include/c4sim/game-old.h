#include <cstdint>
#include <vector>
#include <array>
#include <algorithm>
#include <iostream>

namespace c4sim
{

    // constants
    constexpr int HEIGHT = 6;
    constexpr int WIDTH  = 7;
    constexpr int MAX_MOVES = HEIGHT * WIDTH;
        
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
        std::uint8_t num_moves = 0;
        std::array<std::uint8_t, MAX_MOVES> moves {0};

        // these should go into a struct
        std::array<std::uint64_t, 2> bitboard {};
        std::array<std::uint8_t, WIDTH> heights {0, 7, 14, 21, 28, 35, 42};
        // GameState gameState;
        // std::vector<std::
        std::array<char, 64> board_state {};
        
        GameState gameState;
    public:
        Game() {
            board_state.fill('-');
        }

        void print_moves() {
        for (const auto &i : moves) {
            std::cout << static_cast<int>(i) << ", ";
        }
        std::cout << std::endl;
        }
        
        void generate_state(std::uint64_t bitboard, char symbol) {
            std::uint64_t one = 1;
            for (int i = 0; i < 64; ++i) {
                if ((one << i) & bitboard)
                board_state[i] = symbol;
            }
        }
        
        std::array<char, 64> get_board() {
            generate_state(bitboard[0], 'X');
            generate_state(bitboard[1], 'O');

            return board_state;
        }
        
        void print_board() {
            generate_state(bitboard[0], 'X');
            generate_state(bitboard[1], 'O');
            
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

            // for (int i = 0; i < 64; ++i) {
                //     std::cout << board_state[i];
                // }
                // std::cout << std::endl;
            }
            
        void make_move(std::uint8_t column) {
                
                if (num_moves >= MAX_MOVES)
                return;
            std::uint64_t one = 1;
            std::uint64_t move = one << heights[column]++;
            bitboard[num_moves & one] ^= move;
            moves[num_moves++] = column;
            
            //     moves[num_moves++] = column;
            // else
            //     throw std::runtime_error("Too many moves");
        }
        
        void undo_move() {
            std::uint64_t one = 1;
            std::uint8_t column = moves[--num_moves];
            std::uint64_t move = one << --heights[column];
            bitboard[num_moves & one] ^= move;
        }
        
        bool is_win(std::uint64_t bitboard) {
            const std::array<std::uint8_t, 4> directions {1, 7, 6, 8};
            uint64_t temp_bb;
            for (auto &direction : directions) {
                temp_bb = bitboard & (bitboard >> direction);
                if ((temp_bb & (temp_bb >> 2 * direction)) != 0) return true;
            }
            return false;
        }
        
        std::vector<std::uint8_t> valid_moves() {
            std::uint64_t one = 1;
            std::vector<std::uint8_t> moves {};
            std::uint64_t TOP = 0b1000000'1000000'1000000'1000000'1000000'1000000'1000000;
            for (std::uint8_t col = 0; col < WIDTH; col++) {
                if ((TOP & (one << heights[col])) == 0) moves.push_back(col);
            }
            return moves;
        }
        
        bool is_move_valid(std::uint8_t column) {
            auto valid_moves_temp = valid_moves();
            if (std::count(valid_moves_temp.begin(), valid_moves_temp.end(), column))
            return true;
            else
            return false;
        }
        
        bool game_is_valid() {
            bool winner1 = is_win(bitboard[0]);
            bool winner2 = is_win(bitboard[1]);
            if (winner1 or winner2)
                return false;
            bool move_count = num_moves < MAX_MOVES;
            return move_count;
        }

        bool is_winner1() {
            return is_win(bitboard[0]);
        }
        
        bool is_winner2() {
            return is_win(bitboard[1]);
        }
        
        bool is_player1_move() {
            return not (num_moves & 1L);
        }
    };

    // game::game(/* args */)
    // {
        // }
        
        // game::~game()
    // {
        // }

} // namespace c4sim
    