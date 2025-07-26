package frc.robot.subsystems.v2_Redundancy.superstructure.manipulator;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.InternalLoggedTracer;
import frc.robot.util.PhoenixUtil;

public class V2_RedundancyManipulatorIOTalonFX implements V2_RedundancyManipulatorIO {
  private final TalonFX armTalonFX;
  private final TalonFX rollerTalonFX;

  private final TalonFXConfiguration armConfig;
  private final TalonFXConfiguration rollerConfig;

  private final StatusSignal<Angle> armPositionRotations;
  private final StatusSignal<AngularVelocity> armVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> armAppliedVolts;
  private final StatusSignal<Current> armSupplyCurrentAmps;
  private final StatusSignal<Current> armTorqueCurrentAmps;
  private final StatusSignal<Temperature> armTemperatureCelsius;
  private final StatusSignal<Double> armPositionSetpointRotations;
  private final StatusSignal<Double> armPositionErrorRotations;

  private final StatusSignal<Angle> rollerPositionRotations;
  private final StatusSignal<AngularVelocity> rollerVelocityRotationsPerSecond;
  private final StatusSignal<AngularAcceleration> rollerAccelerationRotationsPerSecondSquared;
  private final StatusSignal<Voltage> rollerAppliedVoltage;
  private final StatusSignal<Current> rollerSupplyCurrentAmps;
  private final StatusSignal<Current> rollerTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerTemperatureCelsius;

  private Rotation2d armPositionGoal;

  private final DynamicMotionMagicVoltage positionControlRequest;
  private final VoltageOut voltageRequest;

  public V2_RedundancyManipulatorIOTalonFX() {
    armTalonFX = new TalonFX(V2_RedundancyManipulatorConstants.ARM_CAN_ID);
    rollerTalonFX = new TalonFX(V2_RedundancyManipulatorConstants.ROLLER_CAN_ID);

    armConfig = new TalonFXConfiguration();
    armConfig.Feedback.SensorToMechanismRatio =
        V2_RedundancyManipulatorConstants.ARM_PARAMETERS.GEAR_RATIO();
    armConfig.CurrentLimits.withSupplyCurrentLimit(
        V2_RedundancyManipulatorConstants.CURRENT_LIMITS.ARM_SUPPLY_CURRENT_LIMIT());
    armConfig.CurrentLimits.withStatorCurrentLimit(
        V2_RedundancyManipulatorConstants.CURRENT_LIMITS.ARM_STATOR_CURRENT_LIMIT());
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.Slot0.kP = V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kP().get();
    armConfig.Slot0.kD = V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kD().get();
    armConfig.Slot0.kS = V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kS().get();
    armConfig.Slot0.kV = V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kV().get();
    armConfig.Slot0.kA = V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kA().get();
    armConfig.Slot1.kP = V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kP().get();
    armConfig.Slot1.kD = V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kD().get();
    armConfig.Slot1.kS = V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kS().get();
    armConfig.Slot1.kV = V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kV().get();
    armConfig.Slot1.kA = V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kA().get();

    armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MAX_ANGLE().getRotations();
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MIN_ANGLE().getRotations();
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    armConfig.MotionMagic.MotionMagicAcceleration =
        V2_RedundancyManipulatorConstants.CONSTRAINTS
            .MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED()
            .get();
    armConfig.MotionMagic.MotionMagicCruiseVelocity =
        V2_RedundancyManipulatorConstants.CONSTRAINTS
            .CRUISING_VELOCITY_ROTATIONS_PER_SECOND()
            .get();

    tryUntilOk(5, () -> armTalonFX.getConfigurator().apply(armConfig, 0.25));

    rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.CurrentLimits.SupplyCurrentLimit =
        V2_RedundancyManipulatorConstants.CURRENT_LIMITS.ROLLER_SUPPLY_CURRENT_LIMIT();
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit =
        V2_RedundancyManipulatorConstants.CURRENT_LIMITS.ROLLER_STATOR_CURRENT_LIMIT();
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tryUntilOk(5, () -> rollerTalonFX.getConfigurator().apply(rollerConfig, 0.25));

    armPositionRotations = armTalonFX.getPosition();
    armVelocityRotationsPerSecond = armTalonFX.getVelocity();
    armAppliedVolts = armTalonFX.getMotorVoltage();
    armSupplyCurrentAmps = armTalonFX.getSupplyCurrent();
    armTorqueCurrentAmps = armTalonFX.getTorqueCurrent();
    armTemperatureCelsius = armTalonFX.getDeviceTemp();
    armPositionGoal = new Rotation2d();
    armPositionSetpointRotations = armTalonFX.getClosedLoopReference();
    armPositionErrorRotations = armTalonFX.getClosedLoopError();

    rollerPositionRotations = rollerTalonFX.getPosition();
    rollerVelocityRotationsPerSecond = rollerTalonFX.getVelocity();
    rollerAccelerationRotationsPerSecondSquared = rollerTalonFX.getAcceleration();
    rollerAppliedVoltage = rollerTalonFX.getMotorVoltage();
    rollerSupplyCurrentAmps = rollerTalonFX.getSupplyCurrent();
    rollerTorqueCurrentAmps = rollerTalonFX.getTorqueCurrent();
    rollerTemperatureCelsius = rollerTalonFX.getDeviceTemp();

    positionControlRequest =
        new DynamicMotionMagicVoltage(
            0,
            V2_RedundancyManipulatorConstants.CONSTRAINTS
                .CRUISING_VELOCITY_ROTATIONS_PER_SECOND()
                .get(),
            V2_RedundancyManipulatorConstants.CONSTRAINTS
                .MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED()
                .get(),
            0);
    voltageRequest = new VoltageOut(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        armPositionRotations,
        armVelocityRotationsPerSecond,
        armAppliedVolts,
        armSupplyCurrentAmps,
        armTorqueCurrentAmps,
        armTemperatureCelsius,
        armPositionSetpointRotations,
        armPositionErrorRotations,
        rollerPositionRotations,
        rollerVelocityRotationsPerSecond,
        rollerAccelerationRotationsPerSecondSquared,
        rollerAppliedVoltage,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemperatureCelsius);

    armTalonFX.optimizeBusUtilization();
    rollerTalonFX.optimizeBusUtilization();

    armTalonFX.setPosition(
        V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MIN_ANGLE().getRotations());

    PhoenixUtil.registerSignals(
        false,
        armPositionRotations,
        armVelocityRotationsPerSecond,
        armAppliedVolts,
        armSupplyCurrentAmps,
        armTorqueCurrentAmps,
        armTemperatureCelsius,
        armPositionSetpointRotations,
        armPositionErrorRotations,
        rollerPositionRotations,
        rollerVelocityRotationsPerSecond,
        rollerAccelerationRotationsPerSecondSquared,
        rollerAppliedVoltage,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemperatureCelsius);
  }

  @Override
  public void updateInputs(V2_RedundancyManipulatorIOInputs inputs) {
    InternalLoggedTracer.reset();
    inputs.armPosition = Rotation2d.fromRotations(armPositionRotations.getValueAsDouble());
    inputs.armVelocityRadiansPerSecond =
        Units.rotationsToRadians(armVelocityRotationsPerSecond.getValueAsDouble());
    inputs.armAppliedVolts = armAppliedVolts.getValueAsDouble();
    inputs.armSupplyCurrentAmps = armSupplyCurrentAmps.getValueAsDouble();
    inputs.armTorqueCurrentAmps = armTorqueCurrentAmps.getValueAsDouble();
    inputs.armTemperatureCelsius = armTemperatureCelsius.getValueAsDouble();
    inputs.armPositionGoal = armPositionGoal;
    inputs.armPositionSetpoint =
        Rotation2d.fromRotations(armPositionSetpointRotations.getValueAsDouble());
    inputs.armPositionError =
        Rotation2d.fromRotations(armPositionErrorRotations.getValueAsDouble());

    inputs.rollerPosition = Rotation2d.fromRotations(rollerPositionRotations.getValueAsDouble());
    inputs.rollerVelocityRadiansPerSecond =
        Units.rotationsToRadians(rollerVelocityRotationsPerSecond.getValueAsDouble());
    inputs.rollerAccelerationRadiansPerSecondSquared =
        Units.rotationsToRadians(rollerAccelerationRotationsPerSecondSquared.getValueAsDouble());
    inputs.rollerAppliedVolts = rollerAppliedVoltage.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentAmps.getValueAsDouble();
    inputs.rollerTorqueCurrentAmps = rollerTorqueCurrentAmps.getValueAsDouble();
    inputs.rollerTemperatureCelsius = rollerTemperatureCelsius.getValueAsDouble();
    InternalLoggedTracer.record("Update Inputs", "Manipulator/TalonFX");
  }

  @Override
  public void setArmVoltage(double volts) {
    InternalLoggedTracer.reset();
    armTalonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
    InternalLoggedTracer.record("Set Arm Voltage", "Manipulator/TalonFX");
  }

  @Override
  public void setRollerVoltage(double volts) {
    InternalLoggedTracer.reset();
    rollerTalonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
    InternalLoggedTracer.record("Set Roller Voltage", "Manipulator/TalonFX");
  }

  @Override
  public void setArmPositionGoal(Rotation2d position) {
    InternalLoggedTracer.reset();
    armPositionGoal = position;
    armTalonFX.setControl(
        positionControlRequest.withPosition(position.getRotations()).withEnableFOC(true));
    InternalLoggedTracer.record("Set Arm Position Goal", "Manipulator/TalonFX");
  }

  @Override
  public void updateSlot0ArmGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    InternalLoggedTracer.reset();
    armConfig.Slot0.kP = kP;
    armConfig.Slot0.kD = kD;
    armConfig.Slot0.kS = kS;
    armConfig.Slot0.kV = kV;
    armConfig.Slot0.kA = kA;
    tryUntilOk(5, () -> armTalonFX.getConfigurator().apply(armConfig, 0.25));
    InternalLoggedTracer.record("Update PID Slot 0", "Manipulator/TalonFX");
  }

  @Override
  public void updateSlot1ArmGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    InternalLoggedTracer.reset();
    armConfig.Slot0.kP = kP;
    armConfig.Slot0.kD = kD;
    armConfig.Slot0.kS = kS;
    armConfig.Slot0.kV = kV;
    armConfig.Slot0.kA = kA;
    tryUntilOk(5, () -> armTalonFX.getConfigurator().apply(armConfig, 0.25));
    InternalLoggedTracer.record("Update PID Slot 1", "Manipulator/TalonFX");
  }

  @Override
  public void updateArmConstraints(double maxAcceleration, double maxVelocity) {
    InternalLoggedTracer.reset();
    armConfig.MotionMagic.MotionMagicAcceleration = maxAcceleration;
    armConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    tryUntilOk(5, () -> armTalonFX.getConfigurator().apply(armConfig, 0.25));
    InternalLoggedTracer.record("Update Constraints", "Manipulator/TalonFX");
  }

  @Override
  public void zeroArmPosition() {
    armTalonFX.setPosition(
        V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MIN_ANGLE().getRotations());
  }

  @Override
  public void armMax() {
    armTalonFX.setPosition(
        V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MAX_ANGLE().getRotations());
  }
}
