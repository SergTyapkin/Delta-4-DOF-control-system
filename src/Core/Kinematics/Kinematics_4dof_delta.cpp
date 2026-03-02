#include <Arduino.h>
#include "Kinematics.h"
#include "../../../src/utils/Logger.h"
#include "../../../src/utils/MathUtils.h"
#include "../../../src/utils/Matrix.h"


// ======= INVERSE ========
Kinematics::Solution Kinematics::solveInverseKinematics(const Vector6& effector_position) {
  Solution solution;

  // Ищем точки шарниров на платформе эффектора
  Vector3 effector_joints_positions[RobotParams::MOTORS_COUNT];
  getEffectorJointsPositions(effector_position, effector_joints_positions);

  for (uint8_t i = 0; i < RobotParams::MOTORS_COUNT; i++) {
    // Получаем решение для каждого шарнира
    solution.joints_positions_z[i] = solveInverseKinematicsForArm(effector_joints_positions[i], i);

    // Проверка решения
    if (!isSolutionPhysical(solution.joints_positions_z[i])) {
      solution.valid = false;
      solution.error_code = 20;
      snprintf(solution.error_message, sizeof(solution.error_message),
               "Non-physical solution for arm %d: %.3f", i, solution.joints_positions_z[i]);
      return solution;
    }

    // Конвертация в угол поворота двигателя
    solution.joints_angles[i] = convertPositionZToJointAngle(solution.joints_positions_z[i]);
  }

  solution.valid = true;
  solution.error_message[0] = '\0';
  return solution;
}

float Kinematics::convertPositionZToJointAngle(const float position_z) {
  const float home_z = 0.0f;
  const float revolutions = (home_z - position_z) / RobotParams::MM_PER_REVOLUTION;
  const float radians = revolutions * MathUtils::TWO_PI;
  return radians;
}

float Kinematics::solveInverseKinematicsForArm(const Vector3& end_arm_joint_position, const uint8_t arm_index) {
  Vector3 start_arm_joint_position = config_.column_joint_positions[arm_index];
  // Проводим через начало рычага вертикальную прямую, а через конец - сферу с радиусом равным длине рычага.
  // 2 перечечения - это точки, в которых начало рычага может быть. Нам нужна только верхняя

  // 1. Находим горизонтальное расстояние от прямой, проведенной через начало рычага до центра сферы из конца рычага
  float dx = start_arm_joint_position.x - end_arm_joint_position.x;
  float dy = start_arm_joint_position.y - end_arm_joint_position.y;
  float horizontalDistanceSq = dx * dx + dy * dy;
  float horizontalDistance = sqrtf(horizontalDistanceSq);

  // 2. Если прямая не пересекает сферу - решения нет
  if (horizontalDistance > config_.arm_length) {
    return NAN;
  }

  // 3. Находим вертикальную полу-длину отрезка пересечения по теореме Пифагора
  float verticalHalfLength = sqrtf(config_.arm_length * config_.arm_length - horizontalDistanceSq);

  // 4. Вычисляем z-координаты точек пересечения
  float z1 = end_arm_joint_position.z + verticalHalfLength;
  //  float z2 = end_arm_joint_position.z - verticalHalfLength; // Нижняя точка нас не интересует

  return z1;
}

void Kinematics::getEffectorJointsPositions(
    const Vector6& effector_pos,
    Vector3 (&result)[RobotParams::MOTORS_COUNT]
) const {
  const Vector3 effector_point = effector_pos.toPosition();
  for (uint8_t i = 0; i < RobotParams::MOTORS_COUNT; i++) {
    Vector3 effector_joint_pos =
        effector_point + Vector3(config_.effector_joint_positions[i].x, config_.effector_joint_positions[i].y, config_.effector_joint_positions[i].z + config_.effector_height);
    Vector3 effector_joint_pos_rotated = Matrix::rotateVectorAroundPoint(
        effector_joint_pos, effector_pos.toOrientation(), effector_point
    );
    result[i] = effector_joint_pos_rotated;
  }
}

// ======= FORWARDS ========
bool Kinematics::solveForwardKinematics(const float joints_angles[RobotParams::MOTORS_COUNT], Vector6& position) {
//  Vector3 guess(0, 0, (Limits::WORKSPACE.min_z + Limits::WORKSPACE.max_z) / 2);
//
//  const uint8_t MAX_ITERATIONS = 50;
//  const float TOLERANCE = 0.001f;
//
//  for (uint8_t iter = 0; iter < MAX_ITERATIONS; iter++) {
//    Solution solution = inverseKinematics(guess);
//
//    if (!solution.valid) {
//      Logger::warning("Forward kinematics: invalid solution at iteration %d", iter);
//      return false;
//    }
//
//    float error[RobotParams::MOTORS_COUNT];
//    float max_error = 0;
//    for (int i = 0; i < RobotParams::MOTORS_COUNT; i++) {
//      error[i] = angles[i] - solution.angles[i];
//      if (fabs(error[i]) > max_error) {
//        max_error = fabs(error[i]);
//      }
//    }
//
//    if (max_error < TOLERANCE) {
//      position = guess;
//      return true;
//    }
//
//    float jacobian[RobotParams::MOTORS_COUNT][3];
//    if (!computeAnglesJacobian(guess, jacobian)) {
//      Logger::warning("Forward kinematics: failed to compute Jacobian");
//      return false;
//    }
//
//    // В 4-DOF системе у вас больше нет квадратной матрицы Якобиана (она 4×3),
//    // поэтому нужна псевдообратная матрица через (J^T * J)^-1 * J^T (метод наименьших квадратов)
//    // J имеет размерность [MOTORS_COUNT x 3], потому вычисляем J^T * J [3x3]
//    float JTJ[3][3];
//    Matrix::computeJTJ(jacobian, JTJ);
//
//    // Вычисляем определитель JTJ (3x3)
//    float det = Matrix::determinant3x3(JTJ);
//    if (fabs(det) < 0.0001f) {
//      Logger::warning("Forward kinematics: singular Jacobian matrix");
//      return false;
//    }
//
//    // Вычисляем обратную матрицу (JTJ)^-1 [3x3]
//    float invJTJ[3][3];
//    if (!Matrix::inverse3x3(JTJ, invJTJ)) {
//        return false;
//    }
//
//    float Jt[3][4];
//    Matrix::transposeJacobian(jacobian, Jt);
//
//    // delta_pos = invJTJ * Jt * error
//    float delta_pos[4] = {0};
//    for (int i = 0; i < 4; i++) {
//        for (int j = 0; j < 3; j++) {
//            delta_pos[i] += Jt[j][i] * error[j];
//        }
//    }
//
//    guess.x += delta_pos[0] * 0.5f;
//    guess.y += delta_pos[1] * 0.5f;
//    guess.z += delta_pos[2] * 0.5f;
//  }
//
//  Logger::warning("Forward kinematics: did not converge");
  Logger::warning("Forward kinematics: not implemented yet");
  return false;
}


// ======= CHECKS ========
Kinematics::SingularityType Kinematics::checkSingularity(
    const Vector6& position,
    const float joints_positions_z[RobotParams::MOTORS_COUNT]
) {
  // TODO: Вычисление сингулярностей
  //  for (uint8_t i = 0; i < RobotParams::MOTORS_COUNT; i++) {
  //    Vector3 upper_joint = getUpperJointPosition(angles[i], i);
  //    Vector3 effector_joint = getEffectorJointPosition(position, i);
  //
  //    float distance = upper_joint.distanceTo(effector_joint);
  //
  //    if (fabs(distance - (config_.arm_length + config_.forearm_length)) < 1.0f) {
  //      return SING_EXTENDED;
  //    }
  //
  //    if (fabs(distance - fabs(config_.arm_length - config_.forearm_length)) < 1.0f) {
  //      return SING_RETRACTED;
  //    }
  //  }

  return SING_NONE;
}

bool Kinematics::checkCollisions(
    const Vector6& position,
    const float joints_positions_z[RobotParams::MOTORS_COUNT]
) const {
  // TODO: Проверка на коллизии
//  for (uint8_t i = 0; i < RobotParams::MOTORS_COUNT; i++) {
//    Vector3 effector_joint_pos = solver_.getEffectorJointPosition(position, i);
//    Vector3 upper_joint_pos = solver_.getUpperJointPosition(angles[i], i);
//
//    float distance = upper_joint_pos.distanceTo(effector_joint_pos);
//    float min_distance = fabs(config.arm_length - config.forearm_length);
//    float max_distance = config.arm_length + config.forearm_length;
//
//    if (
//        distance < min_distance + COLLISIONS_CHECK_OFFSET_MM ||
//        distance > max_distance - COLLISIONS_CHECK_OFFSET_MM
//    ) {
//      return false;
//    }
//  }

  return true;
}
