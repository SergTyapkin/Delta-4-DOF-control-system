#include "RobotState.h"

RobotState::RobotState() :
    status(STATUS_IDLE),
    effector_position(0, 0, 0),
    target_position(0, 0, 0),
    current_velocity(0),
    target_velocity(0),
    movement_progress(0),
    error_code(0),
    timestamp(0),
    is_homed(false),
    is_moving(false),
    is_paused(false),
    is_emergency(false) {

  joint_positions[0] = joint_positions[1] = joint_positions[2] = 0;
  joint_velocities[0] = joint_velocities[1] = joint_velocities[2] = 0;
  target_joints[0] = target_joints[1] = target_joints[2] = 0;
  error_message[0] = '\0';
}

String RobotState::serialize() const {
  String json = "{";

  json += "\"status\":" + String((int)status) + ",";

  json += "\"position\":{";
  json += "\"x\":" + String(effector_position.x, 2) + ",";
  json += "\"y\":" + String(effector_position.y, 2) + ",";
  json += "\"z\":" + String(effector_position.z, 2) + "},";

  json += "\"joints\":[";
  json += String(joint_positions[0], 4) + ",";
  json += String(joint_positions[1], 4) + ",";
  json += String(joint_positions[2], 4) + "],";

  json += "\"target\":{";
  json += "\"x\":" + String(target_position.x, 2) + ",";
  json += "\"y\":" + String(target_position.y, 2) + ",";
  json += "\"z\":" + String(target_position.z, 2) + "},";

  json += "\"velocity\":{";
  json += "\"current\":" + String(current_velocity, 2) + ",";
  json += "\"target\":" + String(target_velocity, 2) + "},";

  json += "\"progress\":" + String(movement_progress, 3) + ",";
  json += "\"is_homed\":" + String(is_homed ? "true" : "false") + ",";
  json += "\"is_moving\":" + String(is_moving ? "true" : "false") + ",";
  json += "\"is_paused\":" + String(is_paused ? "true" : "false") + ",";
  json += "\"is_emergency\":" + String(is_emergency ? "true" : "false") + ",";
  json += "\"error_code\":" + String(error_code) + ",";
  json += "\"timestamp\":" + String(timestamp);

  json += "}";
  return json;
}

void RobotState::print() const {
  Serial.println("=== Robot State ===");
  Serial.print("Status: ");
  switch (status) {
    case STATUS_IDLE: Serial.println("IDLE"); break;
    case STATUS_HOMING: Serial.println("HOMING"); break;
    case STATUS_MOVING: Serial.println("MOVING"); break;
    case STATUS_PAUSED: Serial.println("PAUSED"); break;
    case STATUS_ERROR: Serial.println("ERROR"); break;
    case STATUS_EMERGENCY_STOP: Serial.println("EMERGENCY_STOP"); break;
    case STATUS_CALIBRATING: Serial.println("CALIBRATING"); break;
  }

  Serial.print("Position: (");
  Serial.print(effector_position.x, 1);
  Serial.print(", ");
  Serial.print(effector_position.y, 1);
  Serial.print(", ");
  Serial.print(effector_position.z, 1);
  Serial.println(") mm");

  Serial.print("Joints: (");
  Serial.print(joint_positions[0] * 57.2958f, 2);
  Serial.print(", ");
  Serial.print(joint_positions[1] * 57.2958f, 2);
  Serial.print(", ");
  Serial.print(joint_positions[2] * 57.2958f, 2);
  Serial.println(") deg");

  Serial.print("Velocity: ");
  Serial.print(current_velocity, 1);
  Serial.print(" mm/s (target: ");
  Serial.print(target_velocity, 1);
  Serial.println(" mm/s)");

  Serial.print("Progress: ");
  Serial.print(movement_progress * 100, 1);
  Serial.println("%");

  Serial.print("Homed: ");
  Serial.print(is_homed ? "YES" : "NO");
  Serial.print(", Moving: ");
  Serial.print(is_moving ? "YES" : "NO");
  Serial.print(", Paused: ");
  Serial.print(is_paused ? "YES" : "NO");
  Serial.print(", Emergency: ");
  Serial.println(is_emergency ? "YES" : "NO");

  if (error_code != 0) {
    Serial.print("Error: ");
    Serial.print(error_code);
    Serial.print(" - ");
    Serial.println(error_message);
  }
}
