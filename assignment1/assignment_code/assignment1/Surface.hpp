#ifndef SURFACE_H_
#define SURFACE_H_

#include <string>
#include <vector>

#include "gloo/SceneNode.hpp"

namespace GLOO {
class Surface : public SceneNode {
    public:
        Surface();
        void Update(double delta_time) override;
};
} // namespace GLOO

#endif
