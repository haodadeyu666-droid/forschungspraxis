// Injection with the needle tilted 45 deg from vertical.
// Per program: start pose, flange-to-tip offset and depth. Speeds/limits are shared, see injection_common.h.
// Teach a pose with:  ./injection_45deg --print-pose
#include "injection_common.h"

int main(int argc, char** argv) {
  InjectionConfig config;
  config.program_name = "45deg";
  config.start_position = {0.38, 0.0, 0.26};  // needle tip [m]
  config.start_rpy_deg = {135.0, 0.0, 0.0};  // TCP z axis = [0, -0.707, -0.707]: down, tilted to -y
  config.flange_to_tip = 0.14;  // flange -> needle tip [m]
  config.depth = 0.03;          // insertion depth for this test [m]
  return run_injection_program(config, argc, argv);
}
