#include "Surface.hpp"

#include "gloo/InputManager.hpp"

namespace GLOO {
Surface::Surface() {
}

void Surface::Update(double delta_time) {
    static bool prev_released = true;

    if (InputManager::GetInstance().IsKeyPressed('M')) {
        if (prev_released) {
            size_t child_count = GetChildrenCount();
            for (size_t i = 0; i < child_count; i++) {
                SceneNode& child = GetChild(i);
                child.Update(delta_time);
            }
        }
        prev_released = false;
    }
}
} // namespace GLOO
