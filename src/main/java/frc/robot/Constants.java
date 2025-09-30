// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final boolean TUNING_MODE = false;
  public static final double LOOP_PERIOD_SECONDS = 0.02;
  public static final RobotType ROBOT = RobotType.V2_REDUNDANCY;

  public static Mode getMode() {
    switch (ROBOT) {
      case V2_REDUNDANCY:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case V2_REDUNDANCY_SIM:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static enum RobotType {
    V2_REDUNDANCY,
    V2_REDUNDANCY_SIM
  }

  public static void main(String... args) {
    if (ROBOT == RobotType.V2_REDUNDANCY_SIM) {
      System.err.println("Cannot deploy, invalid mode selected: " + ROBOT.toString());
      System.exit(1);
    }
  }
}
