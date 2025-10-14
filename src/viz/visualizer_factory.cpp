#include "viz/visualizer_interface.hpp"

#ifdef ENABLE_VISUALIZATION
#include "viz/imgui_visualizer.hpp"
#endif

namespace navsim {
namespace viz {

std::unique_ptr<IVisualizer> createVisualizer(bool enable_gui) {
#ifdef ENABLE_VISUALIZATION
  if (enable_gui) {
    return std::make_unique<ImGuiVisualizer>();
  }
#endif
  return std::make_unique<NullVisualizer>();
}

} // namespace viz
} // namespace navsim

