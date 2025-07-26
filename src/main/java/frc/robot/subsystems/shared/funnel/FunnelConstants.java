package frc.robot.subsystems.shared.funnel;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class FunnelConstants {
  public static final int CLAP_DADDY_MOTOR_ID;
  public static final int ROLLER_MOTOR_ID;
  public static final int CORAL_SENSOR_ID;
  public static final int CLAP_DADDY_CANCODER_ID;
  public static final double CLAP_DADDY_MOTOR_GEAR_RATIO;
  public static final double ROLLER_MOTOR_GEAR_RATIO;
  public static final double CLAP_DADDY_CANCODER_GEAR_RATIO;
  public static final Rotation2d CANCODER_ABSOLUTE_OFFSET_RADIANS;
  public static final InvertedValue CLAP_DADDY_INVERTED;
  public static final InvertedValue ROLLER_INVERTED;

  public static final CurrentLimits CURRENT_LIMITS;
  public static final Thresholds ANGLE_THRESHOLDS;
  public static final MotorParameters CLAP_DADDY_PARAMS;
  public static final MotorParameters ROLLER_PARAMS;

  public static final Gains CLAP_DADDY_MOTOR_GAINS;
  public static final Constraints CLAP_DADDY_MOTOR_CONSTRAINTS;

  static {
    switch (Constants.ROBOT) {
      case V2_REDUNDANCY:
      case V2_REDUNDANCY_SIM:
      default:
        CLAP_DADDY_MOTOR_ID = 41;
        ROLLER_MOTOR_ID = 40;
        CORAL_SENSOR_ID = 0;
        CLAP_DADDY_CANCODER_ID = 40;
        CLAP_DADDY_MOTOR_GEAR_RATIO = 34.0;
        ROLLER_MOTOR_GEAR_RATIO = 5.33333333;
        CLAP_DADDY_CANCODER_GEAR_RATIO = 1.5;
        CANCODER_ABSOLUTE_OFFSET_RADIANS = Rotation2d.fromRadians(-4.651029748869264);
        CLAP_DADDY_INVERTED = InvertedValue.CounterClockwise_Positive;
        ROLLER_INVERTED = InvertedValue.Clockwise_Positive;

        CURRENT_LIMITS = new CurrentLimits(20.0, 20.0, 40.0, 40.0);
        ANGLE_THRESHOLDS = new Thresholds(Units.degreesToRadians(95.0), 0.0);
        CLAP_DADDY_PARAMS = new MotorParameters(DCMotor.getKrakenX60(1), 0.0042);
        ROLLER_PARAMS = new MotorParameters(DCMotor.getKrakenX60(1), 0.0042);

        switch (Constants.getMode()) {
          case REAL:
          case REPLAY:
          default:
            CLAP_DADDY_MOTOR_GAINS =
                new Gains(
                    new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kP", 125.0),
                    new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kD", 5.0),
                    new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kS", 0.224),
                    new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kV", 0.0),
                    new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kA", 0.0));
            CLAP_DADDY_MOTOR_CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Funnel/Clap Daddy Motor/Max Acceleration", 200.0),
                    new LoggedTunableNumber("Funnel/Clap Daddy Motor/Max Velocity", 200.0),
                    new LoggedTunableNumber("Funnel/Goal Tolerance", 0.0));
            break;
          case SIM:
            CLAP_DADDY_MOTOR_GAINS =
                new Gains(
                    new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kP", 40),
                    new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kD", 0.0),
                    new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kS", 0.0),
                    new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kV", 0.0),
                    new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kA", 0.0));
            CLAP_DADDY_MOTOR_CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Funnel/Clap Daddy Motor/Max Acceleration", 100.0),
                    new LoggedTunableNumber("Funnel/Clap Daddy Motor/Max Velocity", 100.0),
                    new LoggedTunableNumber("Funnel/Goal Tolerance", 0.0));
            break;
        }
        break;
    }
  }

  public static final record CurrentLimits(
      double CLAP_DADDY_SUPPLY_CURRENT_LIMIT,
      double ROLLER_SUPPLY_CURRENT_LIMIT,
      double CLAP_DADDY_STATOR_CURRENT_LIMIT,
      double ROLLER_STATOR_CURRENT_LIMIT) {}

  public static final record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public static final record Thresholds(double MAX_ANGLE_RADIANS, double MIN_ANGLE_RADIANS) {}

  public static final record Constraints(
      LoggedTunableNumber MAX_ACCELERATION,
      LoggedTunableNumber MAX_VELOCITY,
      LoggedTunableNumber GOAL_TOLERANCE) {}

  public static final record MotorParameters(DCMotor MOTOR, double MOMENT_OF_INERTIA) {}

  @RequiredArgsConstructor
  public enum FunnelState {
    OPENED(Rotation2d.fromDegrees(60.0)),
    CLOSED(Rotation2d.fromDegrees(93.0)),
    CLIMB(Rotation2d.fromDegrees(0.0));

    private final Rotation2d angle;

    public Rotation2d getAngle() {
      return angle;
    }
  }

  @RequiredArgsConstructor
  public enum FunnelRollerState {
    STOP(0.0),
    INTAKE(12.0),
    OUTTAKE(-12.0);

    private final double voltage;

    public double getVoltage() {
      return voltage;
    }
  }
}
