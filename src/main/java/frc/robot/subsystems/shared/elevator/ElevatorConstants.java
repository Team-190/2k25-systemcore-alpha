// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shared.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.util.LoggedTunableNumber;
import java.util.Map;
import lombok.RequiredArgsConstructor;

public class ElevatorConstants {
  public static final int ELEVATOR_CAN_ID;
  public static final double ELEVATOR_GEAR_RATIO;
  public static final double DRUM_RADIUS;

  public static final double ELEVATOR_SUPPLY_CURRENT_LIMIT;
  public static final double ELEVATOR_STATOR_CURRENT_LIMIT;

  public static final ElevatorParameters ELEVATOR_PARAMETERS;
  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;
  public static final Gains STOW_GAINS;
  public static final Constraints STOW_CONSTRAINTS;

  public static final Map<ReefState, ElevatorPositions> REEF_STATE_ELEVATOR_POSITION_MAP;

  static {
    REEF_STATE_ELEVATOR_POSITION_MAP =
        Map.ofEntries(
            Map.entry(ReefState.STOW, ElevatorPositions.STOW),
            Map.entry(ReefState.CORAL_INTAKE, ElevatorPositions.CORAL_INTAKE),
            Map.entry(ReefState.ALGAE_FLOOR_INTAKE, ElevatorPositions.ALGAE_INTAKE),
            Map.entry(ReefState.ALGAE_MID, ElevatorPositions.ALGAE_MID),
            Map.entry(ReefState.ALGAE_INTAKE_TOP, ElevatorPositions.ALGAE_INTAKE_TOP),
            Map.entry(ReefState.ALGAE_INTAKE_BOTTOM, ElevatorPositions.ALGAE_INTAKE_BOT),
            Map.entry(ReefState.L1, ElevatorPositions.L1),
            Map.entry(ReefState.L2, ElevatorPositions.L2),
            Map.entry(ReefState.L3, ElevatorPositions.L3),
            Map.entry(ReefState.L4, ElevatorPositions.L4),
            Map.entry(ReefState.L4_PLUS, ElevatorPositions.L4_PLUS),
            Map.entry(ReefState.ALGAE_SCORE, ElevatorPositions.ALGAE_SCORE));

    switch (Constants.ROBOT) {
      case V2_REDUNDANCY:
      case V2_REDUNDANCY_SIM:
      default:
        ELEVATOR_CAN_ID = 20;
        ELEVATOR_GEAR_RATIO = 4.0;
        DRUM_RADIUS = Units.inchesToMeters(2.256 / 2.0);

        ELEVATOR_SUPPLY_CURRENT_LIMIT = 40;
        ELEVATOR_STATOR_CURRENT_LIMIT = 80;

        ELEVATOR_PARAMETERS =
            new ElevatorParameters(
                DCMotor.getKrakenX60Foc(2), 6.803886, 0.0, 1.43 + Units.inchesToMeters(0.5), 2);

        switch (Constants.getMode()) {
          case REAL:
          case REPLAY:
          default:
            GAINS =
                new Gains(
                    new LoggedTunableNumber("Elevator/Gains/kP", 2.0),
                    new LoggedTunableNumber("Elevator/Gains/kD", 0.1),
                    new LoggedTunableNumber("Elevator/Gains/kS", 0.225),
                    new LoggedTunableNumber("Elevator/Gains/kG", 0.075),
                    new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kA", 0.0));
            CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Elevator/Max Acceleration", 16.0),
                    new LoggedTunableNumber("Elevator/Cruising Velocity", 16.0),
                    new LoggedTunableNumber("Elevator/Goal Tolerance", 0.02));
            STOW_GAINS =
                new Gains(
                    new LoggedTunableNumber("Elevator/Stow Gains/kP", 2.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kD", 0.1),
                    new LoggedTunableNumber("Elevator/Stow Gains/kS", 0.225),
                    new LoggedTunableNumber("Elevator/Stow Gains/kG", 0.075),
                    new LoggedTunableNumber("Elevator/Stow Gains/kV", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kA", 0.0));
            STOW_CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Elevator/Stow Max Acceleration", 16.0),
                    new LoggedTunableNumber("Elevator/Stow Cruising Velocity", 16.0),
                    new LoggedTunableNumber("Elevator/Stow Goal Tolerance", 0.02));
            break;
          case SIM:
            GAINS =
                new Gains(
                    new LoggedTunableNumber("Elevator/Gains/kP", 20.0),
                    new LoggedTunableNumber("Elevator/Gains/kD", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kS", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kG", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kA", 0.0));
            CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Elevator/Max Acceleration", 101.078594),
                    new LoggedTunableNumber("Elevator/Cruising Velocity", 11.329982),
                    new LoggedTunableNumber("Elevator/Goal Tolerance", 0.02));
            STOW_GAINS =
                new Gains(
                    new LoggedTunableNumber("Elevator/Stow Gains/kP", 20.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kD", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kS", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kG", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kV", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kA", 0.0));
            STOW_CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Elevator/Stow Max Acceleration", 101.078594),
                    new LoggedTunableNumber("Elevator/Stow Cruising Velocity", 11.329982),
                    new LoggedTunableNumber("Elevator/Stow Goal Tolerance", 0.02));
            break;
        }
        break;
    }
  }

  public static record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kG,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public static record Constraints(
      LoggedTunableNumber maxAccelerationMetersPerSecondSquared,
      LoggedTunableNumber cruisingVelocityMetersPerSecond,
      LoggedTunableNumber goalToleranceMeters) {}

  public static record ElevatorParameters(
      DCMotor ELEVATOR_MOTOR_CONFIG,
      double CARRIAGE_MASS_KG,
      double MIN_HEIGHT_METERS,
      double MAX_HEIGHT_METERS,
      int NUM_MOTORS) {}

  @RequiredArgsConstructor
  public static enum ElevatorPositions {
    STOW(0.0),
    CORAL_INTAKE(0.0),
    ALGAE_INTAKE(0.2161583093038944 + Units.inchesToMeters(1)),
    ALGAE_MID(0.7073684509805078),
    ALGAE_INTAKE_TOP(1.17 - Units.inchesToMeters(8)),
    ALGAE_INTAKE_BOT(0.79 - Units.inchesToMeters(8)),
    ASS_TOP(1.2),
    ASS_BOT(0.82),
    L1(0.11295250319916351),
    L2(0.37296301250898894),
    L3(0.7606347556550676 + Units.inchesToMeters(1.0)),
    L4(1.3864590139769697 + Units.inchesToMeters(0.5)),
    L4_PLUS(1.3864590139769697 + Units.inchesToMeters(2.0)),
    ALGAE_SCORE(1.3864590139769697 + Units.inchesToMeters(0.5));

    private final double position;

    public double getPosition() {
      return position;
    }
  }
}
