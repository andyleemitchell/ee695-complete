#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <ros_c4msgs/MoveMade.h>
#include <ros_c4msgs/NextMove.h>
#include <cstdint>
#include <array>
#include <vector>
#include <random>
#include <chrono>
#include <iostream>
#include <memory>
#include <cassert>
#include <limits>
#include <string>

constexpr int WIDTH = 7;
constexpr int HEIGHT = 6;
constexpr int MAX_MOVES = WIDTH * HEIGHT;

struct GameState
{
    std::uint64_t red {};
    std::uint64_t yellow {};
    std::array<std::uint8_t, 7> heights {};
};

struct MCTSGameState {
    GameState gameState;
    std::uint8_t num_moves;

    bool is_player1_move() const {
        return (num_moves % 2) == 0;
    }

    std::vector<std::uint8_t> get_valid_moves() const {
        std::vector<std::uint8_t> moves;
        std::uint64_t one = 1;
        std::uint64_t TOP = 0b1000000'1000000'1000000'1000000'1000000'1000000'1000000;
        for (std::uint8_t col = 0; col < WIDTH; col++) {
            if ((TOP & (one << gameState.heights[col])) == 0) moves.push_back(col);
        }
        return moves;
    }

    bool is_win(std::uint64_t bitboard) const {
        const std::array<std::uint8_t, 4> directions {1, 7, 6, 8};
        uint64_t temp_bb;
        for (auto &direction : directions) {
            temp_bb = bitboard & (bitboard >> direction);
            if ((temp_bb & (temp_bb >> (2 * direction))) != 0) return true;
        }
        return false;
    }

     bool is_player1_winner() const {
        return is_win(gameState.red);
    }

    bool is_player2_winner() const {
        return is_win(gameState.yellow);
    }

    bool is_terminal() const {
        return is_player1_winner() || is_player2_winner() || num_moves >= MAX_MOVES;
    }

    void make_move(std::uint8_t move) {
        std::uint64_t one = 1;
        std::uint64_t temp_move = one << gameState.heights[move]++;
        if (is_player1_move()) {
            gameState.red ^= temp_move;
        } else {
            gameState.yellow ^= temp_move;
        }
        num_moves++;
    }
};

class MCTSNode {
public:
    std::unique_ptr<MCTSGameState> gameState;
    MCTSNode* parent;
    std::vector<std::unique_ptr<MCTSNode>> children;
    int visits = 0;
    double wins = 0;
    int move;
    std::uint8_t player;
    int depth = 0;   
    bool debug_mode = false;

    MCTSNode(std::unique_ptr<MCTSGameState> gameState, MCTSNode* parent, int move, std::uint8_t player, bool debug = false)
        : gameState(std::move(gameState)), parent(parent), move(move), player(player), debug_mode(debug)  {
        if (parent) {
            assert(player == (3 - parent->player));
            depth = parent->depth + 1;
            debug_mode = parent->debug_mode;
        }

    }

    bool is_leaf() const {
        return children.empty() || gameState->is_terminal();
    }

    MCTSNode* select_child() {
        double best_value = -std::numeric_limits<double>::infinity();
        MCTSNode* best_child = nullptr;

        for (const auto& child : children) {
            double ucb1 = child->ucb1_value();
            debug_print(depth + 1, "  Child Move: " + std::to_string(child->move) +
                                       ", UCB1: " + std::to_string(ucb1) +
                                       ", Visits: " + std::to_string(child->visits) +
                                       ", Wins: " + std::to_string(child->wins));

            if (ucb1 > best_value) {
                best_value = ucb1;
                best_child = child.get();
            }
        }
        if (best_child) {
            debug_print(depth, "Selected Move: " + std::to_string(best_child->move) +
                                       " (UCB1: " + std::to_string(best_value) + ")");
        }

        return best_child;
    }



    double ucb1_value() const {
        if (visits == 0) {
            return std::numeric_limits<double>::infinity();
        }
        double exploitation = wins / static_cast<double>(visits);
        double exploration = 1.4 * std::sqrt(std::log(parent->visits) / static_cast<double>(visits));
        return exploitation + exploration;
    }

    void expand() {
        debug_print(depth, "Expanding node...");

        if (gameState->is_terminal()) {
              debug_print(depth, "  Node is terminal, cannot expand.");
            return;
        }

        std::vector<std::uint8_t> valid_moves = gameState->get_valid_moves();
        for (const auto& move : valid_moves) {
            auto new_state = std::make_unique<MCTSGameState>(*gameState);
            new_state->make_move(move);
            std::uint8_t next_player = (player == 1) ? 2 : 1;
            children.push_back(std::make_unique<MCTSNode>(std::move(new_state), this, move, next_player, debug_mode));
            debug_print(depth + 1, "  Added child with move: " + std::to_string(move));
        }
    }

    int simulate() {
        debug_print(depth, "Starting simulation...");
        auto temp_state = std::make_unique<MCTSGameState>(*gameState);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uint8_t current_player = (player == 1) ? 2 : 1;

        while (!temp_state->is_terminal()) {
            std::vector<std::uint8_t> valid_moves = temp_state->get_valid_moves();
             if (valid_moves.empty()) {
                debug_print(depth, "  No valid moves in simulation (draw).");
                return 0;
            }
            std::uniform_int_distribution<> distrib(0, valid_moves.size() - 1);
            std::uint8_t random_move = valid_moves[distrib(gen)];
            temp_state->make_move(random_move);
            debug_print(depth, "  Simulation: Player " + std::to_string(current_player) +
                                       " made move " + std::to_string(random_move));

            current_player = (current_player == 1) ? 2 : 1;
        }

        int result;
        if (temp_state->is_player1_winner() && player == 1) {
            result = 1;
        } else if (temp_state->is_player2_winner() && player == 2) {
            result = 1;
        } else if (temp_state->is_player1_winner() && player == 2) {
            result = -1;
        }
         else if (temp_state->is_player2_winner() && player == 1) {
            result = -1;
        } else {
            result = 0;
        }
        debug_print(depth, "Simulation result: " + std::to_string(result));

        return result;
    }


    void backpropagate(int result) {
        MCTSNode* current_node = this;

        debug_print(depth, "Backpropagation started with result: " + std::to_string(result));

        while (current_node != nullptr) {
            current_node->visits++;
            current_node->wins += result;
            debug_print(current_node->depth, "  Backprop: Node Move " +
                                       std::to_string(current_node->move) + ", Player " +
                                       std::to_string(current_node->player) +
                                       ", Visits: " + std::to_string(current_node->visits) +
                                       ", Wins: " + std::to_string(current_node->wins) +
                                       ", Result (this node): " + std::to_string(result));

            result = -result;
            current_node = current_node->parent;
        }
    }

    void debug_print(int indent_level, const std::string& message) {
        if (debug_mode) {
            for (int i = 0; i < indent_level; ++i) {
                std::cout << "--";
            }
            std::cout << message << std::endl;
        }
    }
};

class MCTS {
private:
    std::unique_ptr<MCTSNode> root;
    int current_iteration = 0;

public:
    MCTS(const GameState& initial_state, std::uint8_t num_moves, std::uint8_t player, bool debug = false) {
        auto state = std::make_unique<MCTSGameState>();
        state->gameState = initial_state;
        state->num_moves = num_moves;
        root = std::make_unique<MCTSNode>(std::move(state), nullptr, 0, player, debug);
        root->visits = 1;
    }
    std::uint8_t run(int iterations) {
        for (current_iteration = 1; current_iteration <= iterations; ++current_iteration) {
            if (root->debug_mode) {
                std::cout << "\n--- Iteration " << current_iteration << " ---\n";
            }

            MCTSNode* node = root.get();

            while (!node->is_leaf()) {
                node = node->select_child();
                assert(node != nullptr);
            }

            node->expand();

            int result;
            std::random_device rd;
            std::mt19937 gen(rd());

            if (!node->children.empty()) {
                std::uniform_int_distribution<> distrib(0, node->children.size() - 1);
                int random_child_index = distrib(gen);
                node = node->children[random_child_index].get();
                result = node->simulate();
            } else {
                result = node->simulate();
            }

            node->backpropagate(result);
        }

        MCTSNode* best_node = nullptr;
        int max_visits = -1;
        for (const auto& child : root->children) {
            if (root->debug_mode) {
                std::cout << "Final Stats - Move " << static_cast<int>(child->move)
                          << ": Visits = " << child->visits << ", Wins = " << child->wins << "\n";
            }
            if (child->visits > max_visits) {
                max_visits = child->visits;
                best_node = child.get();
            }
        }

        if (!best_node) {
          if (root->debug_mode){
            std::cout << "No best move found (likely a full board or early termination).\n";
          }
            auto valid_moves = root->gameState->get_valid_moves();
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> distrib(0, valid_moves.size() - 1);
            return valid_moves.empty() ? 0 : valid_moves[distrib(gen)];
        }
        if(root->debug_mode){
          std::cout << "Best move chosen: " << static_cast<int>(best_node->move) << "\n";
        }
        return best_node->move;
    }
};



class MonteCarloTreeSearch {
private:
    int iterations;
    std::uint8_t player;
    bool debug;

public:
    MonteCarloTreeSearch(int iterations = 1000, std::uint8_t player = 1, bool debug = false)
        : iterations(iterations), player(player), debug(debug) {}

    std::uint8_t choose_move(const GameState& boardState,
        const std::vector<std::uint8_t>& valid_moves, std::uint8_t num_moves) {
        
        auto start = std::chrono::high_resolution_clock::now();
        MCTS mcts(boardState, num_moves, player, debug);
        int best_move = mcts.run(iterations);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
        std::cout << duration.count() << "ms\n";
        return static_cast<std::uint8_t>(best_move);
    }
};


class ConnectFourNode {
public:
    ConnectFourNode(): nh(), private_nh("~") {
        move_made_sub_ = nh.subscribe("/c4/wrapper_mcts", 1, &ConnectFourNode::moveMadeCallback, this);

        next_move_pub_ = nh.advertise<ros_c4msgs::NextMove>("/c4/internal_next_move", 1);

        ROS_INFO("MCTS Node started.");
    }

    void spin() {
        ros::spin();
    }

private:
    void moveMadeCallback(const ros_c4msgs::MoveMade::ConstPtr& msg) {
        ROS_INFO("Received move: %s", msg->move_made.c_str());
        
        GameState gameState;
        gameState.red = msg->board_state.player1;
        gameState.yellow = msg->board_state.player2;
        std::array<std::uint8_t, 7> heights {};
        for (auto i = 0; i < 7; i++) {
            heights[i] = msg->board_state.heights[i];
        }
        gameState.heights = heights;
        std::cout << "Heights: ";
        for (const auto& height : gameState.heights) {
            std::cout << static_cast<int>(height) << " ";
        }
        std::cout << std::endl;
        if (msg->move_made == msg->PLAYER2_MOVE) {
            publishNextMove(gameState);
        }
    }

    std::vector<std::uint8_t> get_valid_moves(std::array<std::uint8_t, 7> heights) {
        std::vector<std::uint8_t> moves;
        std::uint64_t one = 1;
        std::uint64_t TOP = 0b1000000'1000000'1000000'1000000'1000000'1000000'1000000;
        for (std::uint8_t col = 0; col < WIDTH; col++) {
            if ((TOP & (one << heights[col])) == 0) moves.push_back(col);
        }
        return moves;
    }

    void publishNextMove(const GameState& gameState) {
        ros_c4msgs::NextMove next_move_msg;

        int difficulty = nh.param("/c4/difficulty", 3);
        int iterations = 2000;
        ROS_INFO("Got difficulty: %d", difficulty);
        if (difficulty == 1) {
            iterations = 20;
        } else if (difficulty == 2) {
            iterations = 70;
        } else {
            iterations = 2000;
        }
        ROS_INFO("Using iterations: %d", iterations);
        MonteCarloTreeSearch mcts(iterations, 2);

        int num_moves = 0;
        std::array<uint8_t, 7> reference_heights = {0, 7, 14, 21, 28, 35, 42};
        // some hacky way of figuring out the number of moves???
        for (int i = 0; i < 7; ++i) {
            int temp = gameState.heights[i] - reference_heights[i];
            std::cout << "temp: " << temp << std::endl;
        }

        std::vector<std::uint8_t> valid_moves = get_valid_moves(gameState.heights);
        std::cout << "Valid moves: ";
        for (const auto& move : valid_moves) {
            std::cout << static_cast<int>(move) << " ";
        }
        std::cout << std::endl;
        std::uint8_t next_move_column = mcts.choose_move(gameState, valid_moves, 0);


        ros::Duration(1).sleep();
        next_move_msg.next_move = next_move_column;
        next_move_pub_.publish(next_move_msg);
        ROS_INFO("Published next move: %d", next_move_column);
        
    }

    ros::Subscriber move_made_sub_;
    ros::Publisher next_move_pub_;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

};



int main(int argc, char** argv) {
    ros::init(argc, argv, "connect_four_node");

    ConnectFourNode c4_node;
    
    srand(time(NULL));

    c4_node.spin();

    return 0;
}
