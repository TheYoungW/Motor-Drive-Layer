#include <cstdlib>

// Pinocchio's default ModelTpl<double> constructor. If a globally loaded ROS
// library is allowed to interpose this symbol, model creation exits immediately.
extern "C" void motor_drive_layer_poisoned_pinocchio_constructor()
    asm("_ZN9pinocchio8ModelTplIdLi0ENS_25JointCollectionDefaultTplEEC1Ev");

extern "C" void motor_drive_layer_poisoned_pinocchio_constructor() {
  std::_Exit(86);
}
