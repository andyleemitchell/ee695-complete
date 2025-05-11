#include "raylib-cpp.hpp"
#include "c4sim/game.h"

#include <iostream>

int main(void) {

    using namespace c4sim;
    // Initialisation
    // ===========================================
    const int screen_width = 700;
    const int screen_height = 600;

    const int offset = 50;
    const float radius = 40;

    raylib::Color text_colour = raylib::Color::Black();
    
    raylib::Color red_piece = raylib::Color::Red();
    raylib::Color yellow_piece = raylib::Color::Yellow();
    
    raylib::Window window(screen_width, screen_height, "c4sim-visual");

    const char* winner1 = "Player 1 wins.";
    const char* winner2 = "Player 2 wins.";
    const int font_size = 40;
    const int padding = 20;

    int text_width = MeasureText(winner1, font_size);

    const int win_rect_width = text_width + 2 * padding;
    const int win_rect_height = font_size + 2 * padding;
    const int win_rect_x = (screen_width - win_rect_width) / 2;
    const int win_rect_y = (screen_height - win_rect_height) / 2;
    raylib::Rectangle win_rect(win_rect_x, win_rect_y, win_rect_width, win_rect_height);

    const int win_text_x = win_rect_x + padding;
    const int win_text_y = win_rect_y + padding;

    bool restart = false;

    SetTargetFPS(60);
    while (true) {
    Game game;
    while (!window.ShouldClose())
    {
        BeginDrawing();
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
        }

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) and game.game_is_valid()) {
            Vector2 mousePos = GetMousePosition();

            int clickedCol = static_cast<int>(mousePos.x) / 100;

            if (game.is_move_valid(clickedCol))
                game.make_move(clickedCol);
        }

        if (game.game_is_valid()) {
            auto mousePos = GetMouseX();
            int hovered_col = static_cast<int>(mousePos) / 100;
            if (game.is_player1_move()) {
                DrawCircle(hovered_col*100 + offset, offset, radius, raylib::Color(255, 110, 110));
            } else {
                DrawCircle(hovered_col*100 + offset, offset, radius, raylib::Color(255, 245, 138));
            }
        }

        if (game.is_winner1()) {
            win_rect.Draw(GREEN);
            DrawRectangleLinesEx(win_rect, 2, BLACK);
            DrawText(winner1, win_text_x, win_text_y, font_size, WHITE);
        }
        if (game.is_winner2()) {
            win_rect.Draw(GREEN);
            DrawRectangleLinesEx(win_rect, 2, BLACK);
            DrawText(winner2, win_text_x, win_text_y, font_size, WHITE);
        }

        if (IsKeyDown(KEY_R) and not game.game_is_valid()) {
            restart = true;
            break;
        }

        if (IsKeyDown(KEY_Q)) {
            restart = false;
            break;
        }
        EndDrawing();

    }
    if (not restart) {
        break;
        window.Close();
    }
    }
    return 0;
}
