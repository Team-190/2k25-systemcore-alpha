// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

public class NTPrefixes {
  public static final String ROBOT_STATE = "RobotState/";
  public static final String OI_DATA = ROBOT_STATE + "Operator Input Data/";
  public static final String POSE_DATA = ROBOT_STATE + "Pose Data/";
  public static final String REEF_DATA = ROBOT_STATE + "Reef Data/";

  public static final String CORAL_DATA = REEF_DATA + "Coral/";
  public static final String ALGAE_DATA = REEF_DATA + "Algae/";

  public static final String CANIVORE_STATUS = ROBOT_STATE + "CANivore Status/";

  public static final String SUPERSTRUCTURE = "Superstructure/";
}
