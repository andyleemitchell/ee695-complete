#include "c4sim/game.h"
#include "c4sim/player.h"
#include "c4sim/players/terminal_user.h"
#include "c4sim/players/raylib_user.h"
#include "c4sim/players/ab_minimax.h"
#include "c4sim/players/random_choice.h"
#include "c4sim/players/mcts.h"

#include <iostream>

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

int main(void) {

    using namespace c4sim;

    // Initialisation
    // ===========================================
    const int screen_width = 700;
    const int screen_height = 700;

    const int offset = 50;
    const float radius = 40;

    raylib::Color text_colour = raylib::Color::Black();
    
    raylib::Color red_piece = raylib::Color::Red();
    raylib::Color yellow_piece = raylib::Color::Yellow();
    
    raylib::Window window(screen_width, screen_height, "c4sim-visual");

    const char* winner1 = "Player 1 wins.";
    const char* winner2 = "Player 2 wins.";
    const char* tie_game = "Tied Game.";
    const int font_size = 40;
    const int padding = 20;

    int text_width = MeasureText(winner1, font_size);

    const int win_rect_width = text_width + 2 * padding;
    const int win_rect_height = font_size + 2 * padding;
    const int win_rect_x = (screen_width - win_rect_width) / 2;
    const int win_rect_y = 5;//(screen_height - win_rect_height) / 2;
    raylib::Rectangle win_rect(win_rect_x, win_rect_y, win_rect_width, win_rect_height);

    const int win_text_x = win_rect_x + padding;
    const int win_text_y = win_rect_y + padding;

    bool restart = false;
    bool game_is_valid = true;

    GameStatus status;

    SetTargetFPS(60);

    while (true) {
        // set up classes for game (these have to be done here as they are smart pointers and trying
        // to delete them and std::move them back to the game after they go out of scope will cause
        // a seg fault.)
        std::shared_ptr<EventQueue> eventQueue = std::make_shared<EventQueue>();
        std::unique_ptr<Player> p1 = std::make_unique<RaylibHuman>(eventQueue);
        // std::unique_ptr<Player> p1 = std::make_unique<MonteCarloTreeSearch>(500, 2);
        std::unique_ptr<Player> p2 = std::make_unique<MonteCarloTreeSearch>(1000, 1);
        // std::unique_ptr<Player> p2 = std::make_unique<RandomChoice>();
        c4sim::Game game(std::move(p1), std::move(p2));

        while (!window.ShouldClose()) {
            BeginDrawing();
            game_is_valid = not(game.is_draw() or game.is_player1_winner() or game.is_player2_winner());
            window.ClearBackground(BLUE);
            {
                auto state = game.get_board();
                for (auto row = 5; row >= 0; --row) {
                    for (auto column = 0; column <= 6; ++column) {
                        auto cell = state[row + ((column) * 7)];
                        if (cell == 'X')
                            DrawCircle((column*100 + offset), (screen_height - row*100 - offset), radius, RED);
                        else if (cell == 'O')
                            DrawCircle((column*100 + offset), (screen_height - row*100 - offset), radius, YELLOW);
                        else
                            DrawCircle((column*100 + offset), (screen_height - row*100 - offset), radius, WHITE);
                    }
                }
                DrawRectangle(0, 0, screen_width, 100, BLACK);
            }
    
            if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
                Vector2 mousePos = GetMousePosition();
                
                std::shared_ptr<MouseClickEvent> event = std::make_shared<MouseClickEvent>(mousePos, MOUSE_BUTTON_LEFT);
                eventQueue->push(event);
            }
    
            if (status == GameStatus::IN_PROGRESS) {
                auto mousePos = GetMouseX();
                int hovered_col = static_cast<int>(mousePos) / 100;
                if (game.is_player1_move()) {
                    DrawCircle(hovered_col*100 + offset, offset, radius, raylib::Color(255, 110, 110));
                } else {
                    DrawCircle(hovered_col*100 + offset, offset, radius, raylib::Color(255, 245, 138));
                }
            }
    
            if (status == GameStatus::PLAYER1_WIN) {
                win_rect.Draw(GREEN);
                DrawRectangleLinesEx(win_rect, 2, BLACK);
                DrawText(winner1, win_text_x, win_text_y, font_size, WHITE);
            }
            if (status == GameStatus::PLAYER2_WIN) {
                win_rect.Draw(GREEN);
                DrawRectangleLinesEx(win_rect, 2, BLACK);
                DrawText(winner2, win_text_x, win_text_y, font_size, WHITE);
            }
            if (status == GameStatus::DRAW) {
                win_rect.Draw(GREEN);
                DrawRectangleLinesEx(win_rect, 2, BLACK);
                DrawText(tie_game, win_text_x, win_text_y, font_size, WHITE);
            }
    
            if (IsKeyDown(KEY_R) and not game_is_valid) {
                restart = true;
                break;
            }
    
            if (IsKeyDown(KEY_Q)) {
                restart = false;
                break;
            }

            status = game.game_tick();

            EndDrawing();
            
            // print_board_state(game.get_board());
        }

        // print the moves at the end of the game
        for (const auto&move : game.get_moves_made()) {
            std::cout << static_cast<int>(move) << "";
        }
        std::cout << "\n";

        if (not restart) {
            break;
            window.Close();
        }
    }
    return 0;

}
