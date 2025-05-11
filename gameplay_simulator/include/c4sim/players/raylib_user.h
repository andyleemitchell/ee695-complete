/**
 * This is for a raylib human user. We use a event queue of mouse clicks to pass click events from
 * outside the game object to the make_move function.
 */

#ifndef RAYLIB_USER_H
#define RAYLIB_USER_H

#include "c4sim/player.h"
#include <raylib-cpp.hpp>

#include <algorithm>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace c4sim {

/**
 * This class represents a mouse click event. The position of the mouse click, and the specific
 * mouse click is recorded.
 * 
 * This could be abstracted into a generic Event class, but for this we only need to detect mouse
 * clicks.
 */
class MouseClickEvent {
    public:
        raylib::Vector2 position;
        int button;
    public:
        MouseClickEvent(raylib::Vector2 pos, int btn) : position(pos), button(btn) {}
};

/**
 * This class represents a queue of events. The condition variable allows waiting for the queue to 
 * be populated without polling.
 */
class EventQueue {
    private:
        std::queue<std::shared_ptr<MouseClickEvent>> m_queue;
        mutable std::mutex m_mutex;
        std::condition_variable m_condition;
    
    public:
        // Add an event to the queue.
        void push(std::shared_ptr<MouseClickEvent> event) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_queue.push(event);
            m_condition.notify_one(); // Notify any waiting threads.
        }
    
        // Get the next event from the queue (blocks if empty).
        std::shared_ptr<MouseClickEvent> pop() {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_condition.wait(lock, [this] { return !m_queue.empty(); }); // Wait until the queue is not empty.
            std::shared_ptr<MouseClickEvent> event = m_queue.front();
            m_queue.pop();
            return event;
        }
    
        // Check if the queue is empty (without blocking).
        bool isEmpty() const {
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_queue.empty();
        }
};

class RaylibHuman : public Player {
    private:
        int choice = -1;
        std::shared_ptr<EventQueue> event_queue; // Holds a reference to the event queue.
        raylib::Vector2 last_click_position;     // Store the last click position.
        bool waiting_for_click = false;         // Flag to indicate we're waiting for a click.

    public:
    
        RaylibHuman(std::shared_ptr<EventQueue> eventQueue) : event_queue(eventQueue) {}
        ~RaylibHuman() = default;

        std::uint8_t choose_move(const GameState& boardState, 
            const std::vector<std::uint8_t>& valid_moves, std::uint8_t num_moves) override {
                waiting_for_click = true; // Set the flag to indicate we are now waiting.
            
                // Process any pending events *before* potentially waiting.
                processEvents();

                //check if waiting.
                if(!waiting_for_click) {
                    choice = static_cast<int>(last_click_position.x) / 100;
                    if (std::count(valid_moves.begin(), valid_moves.end(), choice)) {
                        return choice;
                    } else {
                        return -1;
                    }
                }
                // obviously this will return 255. well maybe not obvious to me considering I spent
                // 30 minutes trying to debug why 255 gets returned all of the time. The checking
                // of a valid column can be (and should be!) handled in the game class.
                return -1;
        }

        // Process events from the queue.
        void processEvents() {
            while (!event_queue->isEmpty() && waiting_for_click) { // only read if waiting.
                std::shared_ptr<MouseClickEvent> event = event_queue->pop();

                // Use dynamic_pointer_cast to check the event type safely. This probably isn't 
                // fully necessary here as the only possible event type is MouseClickEvent, but this
                // could be useful in the future if the possible events are expanded.
                if (std::shared_ptr<MouseClickEvent> clickEvent = std::dynamic_pointer_cast<MouseClickEvent>(event)) {
                    // It's a MouseClickEvent!
                    if (clickEvent->button == MOUSE_BUTTON_LEFT) { // Could also check other buttons.
                        last_click_position = clickEvent->position;
                        waiting_for_click = false; // We got our click.
                    }
                }
            }
        }

};

} // namespace c4sim

#endif//RAYLIB_USER_H
