// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.shared.drive;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.InternalLoggedTracer;
import frc.robot.util.PhoenixUtil;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalonFX;
  private final TalonFX turnTalonFX;
  private final CANcoder cancoder;

  private final TalonFXConfiguration driveConfig;
  private final TalonFXConfiguration turnConfig;
  private final CANcoderConfiguration cancoderConfig;

  private final StatusSignal<Angle> drivePositionRotations;
  private final StatusSignal<AngularVelocity> driveVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveSupplyCurrentAmps;
  private final StatusSignal<Current> driveTorqueCurrentAmps;
  private final StatusSignal<Temperature> driveTemperatureCelcius;
  private final StatusSignal<Double> driveVelocitySetpointRotationsPerSecond;
  private final StatusSignal<Double> driveVelocityErrorRotationsPerSecond;

  private final StatusSignal<Angle> turnAbsolutePositionRotations;
  private final StatusSignal<Angle> turnPositionRotations;
  private final StatusSignal<AngularVelocity> turnVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnSupplyCurrentAmps;
  private final StatusSignal<Current> turnTorqueCurrentAmps;
  private final StatusSignal<Temperature> turnTemperatureCelcius;
  private Rotation2d turnPositionGoal;
  private final StatusSignal<Double> turnPositionSetpointRotations;
  private final StatusSignal<Double> turnPositionErrorRotations;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final TorqueCurrentFOC torqueCurrentRequest;
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest;
  private final MotionMagicTorqueCurrentFOC positionTorqueCurrentRequest;

  private final int id;

  public ModuleIOTalonFX(
      int id,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    driveTalonFX = new TalonFX(constants.DriveMotorId, DriveConstants.DRIVE_CONFIG.canBus());
    turnTalonFX = new TalonFX(constants.SteerMotorId, DriveConstants.DRIVE_CONFIG.canBus());
    cancoder = new CANcoder(constants.EncoderId, DriveConstants.DRIVE_CONFIG.canBus());

    driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = Constants.LOOP_PERIOD_SECONDS;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> driveTalonFX.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalonFX.setPosition(0.0, 0.25));

    turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    turnConfig.Feedback.FeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default -> throw new IllegalArgumentException(
              "Unexpected value: " + constants.FeedbackSource);
        };
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = 40.0;
    turnConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = Constants.LOOP_PERIOD_SECONDS;
    turnConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnTalonFX.getConfigurator().apply(turnConfig, 0.25));

    cancoderConfig = constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));

    drivePositionRotations = driveTalonFX.getPosition();
    driveVelocityRotationsPerSecond = driveTalonFX.getVelocity();
    driveAppliedVolts = driveTalonFX.getMotorVoltage();
    driveSupplyCurrentAmps = driveTalonFX.getSupplyCurrent();
    driveTorqueCurrentAmps = driveTalonFX.getTorqueCurrent();
    driveTemperatureCelcius = driveTalonFX.getDeviceTemp();
    driveVelocitySetpointRotationsPerSecond = driveTalonFX.getClosedLoopReference();
    driveVelocityErrorRotationsPerSecond = driveTalonFX.getClosedLoopError();

    turnAbsolutePositionRotations = cancoder.getAbsolutePosition();
    turnPositionRotations = turnTalonFX.getPosition();
    turnVelocityRotationsPerSecond = turnTalonFX.getVelocity();
    turnAppliedVolts = turnTalonFX.getMotorVoltage();
    turnSupplyCurrentAmps = turnTalonFX.getSupplyCurrent();
    turnTorqueCurrentAmps = turnTalonFX.getTorqueCurrent();
    turnTemperatureCelcius = turnTalonFX.getDeviceTemp();
    turnPositionGoal = new Rotation2d();
    turnPositionSetpointRotations = turnTalonFX.getClosedLoopReference();
    turnPositionErrorRotations = turnTalonFX.getClosedLoopError();

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalonFX.getPosition());
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalonFX.getPosition());

    torqueCurrentRequest = new TorqueCurrentFOC(0.0);
    velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);
    positionTorqueCurrentRequest = new MotionMagicTorqueCurrentFOC(0.0);

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.ODOMETRY_FREQUENCY,
        drivePositionRotations,
        turnPositionRotations,
        turnAbsolutePositionRotations);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocityRotationsPerSecond,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveTorqueCurrentAmps,
        driveTemperatureCelcius,
        driveVelocitySetpointRotationsPerSecond,
        driveVelocityErrorRotationsPerSecond,
        turnVelocityRotationsPerSecond,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnTorqueCurrentAmps,
        turnTemperatureCelcius,
        turnPositionSetpointRotations,
        turnPositionErrorRotations);

    driveTalonFX.optimizeBusUtilization();
    turnTalonFX.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        true,
        drivePositionRotations,
        turnPositionRotations,
        turnAbsolutePositionRotations,
        driveVelocityRotationsPerSecond,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveTorqueCurrentAmps,
        driveTemperatureCelcius,
        driveVelocitySetpointRotationsPerSecond,
        driveVelocityErrorRotationsPerSecond,
        turnVelocityRotationsPerSecond,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnTorqueCurrentAmps,
        turnTemperatureCelcius,
        turnPositionSetpointRotations,
        turnPositionErrorRotations);

    this.id = id;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    InternalLoggedTracer.reset();
    inputs.drivePositionRadians =
        Units.rotationsToRadians(drivePositionRotations.getValueAsDouble());
    inputs.driveVelocityRadiansPerSecond =
        Units.rotationsToRadians(driveVelocityRotationsPerSecond.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrentAmps.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrentAmps.getValueAsDouble();
    inputs.driveTemperatureCelcius = driveTemperatureCelcius.getValueAsDouble();
    inputs.driveVelocitySetpointRadiansPerSecond =
        Units.rotationsToRadians(driveVelocitySetpointRotationsPerSecond.getValueAsDouble());
    inputs.driveVelocityErrorRadiansPerSecond =
        Units.rotationsToRadians(driveVelocityErrorRotationsPerSecond.getValueAsDouble());

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePositionRotations.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPositionRotations.getValueAsDouble());
    inputs.turnVelocityRadiansPerSecond =
        Units.rotationsToRadians(turnVelocityRotationsPerSecond.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrentAmps.getValueAsDouble();
    inputs.turnTorqueCurrentAmps = turnTorqueCurrentAmps.getValueAsDouble();
    inputs.turnTemperatureCelcius = turnTemperatureCelcius.getValueAsDouble();
    inputs.turnPositionGoal = turnPositionGoal;
    inputs.turnPositionSetpoint =
        Rotation2d.fromRotations(turnPositionSetpointRotations.getValueAsDouble());
    inputs.turnPositionError =
        Rotation2d.fromRotations(turnPositionErrorRotations.getValueAsDouble());

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRadians =
        drivePositionQueue.stream().mapToDouble(value -> Units.rotationsToRadians(value)).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    InternalLoggedTracer.record(
        "Update Inputs", "Drive/Modules/" + Integer.toString(id) + "/TalonFX");

    InternalLoggedTracer.reset();
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
    InternalLoggedTracer.record(
        "Reset Queues", "Drive/Modules/" + Integer.toString(id) + "/TalonFX");
  }

  @Override
  public void setDriveAmps(double currentAmps) {
    InternalLoggedTracer.reset();
    driveTalonFX.setControl(torqueCurrentRequest.withOutput(currentAmps));
    InternalLoggedTracer.record(
        "Set Drive Amps", "Drive/Modules/" + Integer.toString(id) + "/TalonFX");
  }

  @Override
  public void setTurnAmps(double currentAmps) {
    InternalLoggedTracer.reset();
    turnTalonFX.setControl(torqueCurrentRequest.withOutput(currentAmps));
    InternalLoggedTracer.record(
        "Set Turn Amps", "Drive/Modules/" + Integer.toString(id) + "/TalonFX");
  }

  @Override
  public void setDriveVelocity(double velocityRadiansPerSecond, double currentFeedforward) {
    InternalLoggedTracer.reset();
    driveTalonFX.setControl(
        velocityTorqueCurrentRequest
            .withVelocity(Units.radiansToRotations(velocityRadiansPerSecond))
            .withFeedForward(currentFeedforward));
    InternalLoggedTracer.record(
        "Set Drive Velocity", "Drive/Modules/" + Integer.toString(id) + "/TalonFX");
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    InternalLoggedTracer.reset();
    turnTalonFX.setControl(positionTorqueCurrentRequest.withPosition(rotation.getRotations()));
    InternalLoggedTracer.record(
        "Set Turn Position", "Drive/Modules/" + Integer.toString(id) + "/TalonFX");
  }

  @Override
  public void setPID(double drive_Kp, double drive_Kd, double turn_Kp, double turn_Kd) {
    InternalLoggedTracer.reset();
    driveConfig.Slot0.kP = drive_Kp;
    driveConfig.Slot0.kD = drive_Kd;
    turnConfig.Slot0.kP = turn_Kp;
    turnConfig.Slot0.kD = turn_Kd;
    tryUntilOk(5, () -> driveTalonFX.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> turnTalonFX.getConfigurator().apply(turnConfig, 0.25));
    InternalLoggedTracer.record("Set PID", "Drive/Modules/" + Integer.toString(id) + "/TalonFX");
  }

  @Override
  public void setFeedforward(double drive_Ks, double drive_Kv) {
    InternalLoggedTracer.reset();
    driveConfig.Slot0.kS = drive_Ks;
    driveConfig.Slot0.kV = drive_Kv;
    tryUntilOk(5, () -> driveTalonFX.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> turnTalonFX.getConfigurator().apply(turnConfig, 0.25));
    InternalLoggedTracer.record(
        "Set Feedforward", "Drive/Modules/" + Integer.toString(id) + "/TalonFX");
  }
}
