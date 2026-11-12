// Injection with the needle pointing vertically down.
// Per program: start pose, flange-to-tip offset and depth. Speeds/limits are shared, see injection_common.h.
// Teach a pose with:  ./injection_0deg --print-pose
#include "injection_common.h"

int main(int argc, char** argv) {
  InjectionConfig config;
  config.program_name = "0deg";
  config.start_position = {0.38, 0.0, 0.50};  // needle tip [m]
  config.start_rpy_deg = {180.0, 0.0, 0.0};   // TCP z axis = -z base (straight down)
  config.flange_to_tip = 0.15;  // flange -> needle tip [m]
  config.depth = 0.04;          // insertion depth for this test [m]
  return run_injection_program(config, argc, argv);
}
