package frc.robot.subsystems.shared.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.shared.drive.TunerConstantsV1_StackUp;
import frc.robot.util.InternalLoggedTracer;
import frc.robot.util.PhoenixUtil;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX talonFX;
  private final TalonFXConfiguration config;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> temperatureCelsius;

  private final DigitalInput redundantSwitchOne;
  private final DigitalInput redundantSwitchTwo;

  private final VoltageOut voltageRequest;

  public ClimberIOTalonFX() {
    talonFX = new TalonFX(ClimberConstants.MOTOR_ID, TunerConstantsV1_StackUp.kCANBus);
    config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit =
        ClimberConstants.CURRENT_LIMITS.SUPPLY_CURRENT_LIMIT();
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit =
        ClimberConstants.CURRENT_LIMITS.STATOR_CURRENT_LIMIT();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

    // Initialize status signals in standardized order
    positionRotations = talonFX.getPosition();
    velocityRotationsPerSecond = talonFX.getVelocity();
    appliedVolts = talonFX.getMotorVoltage();
    supplyCurrentAmps = talonFX.getSupplyCurrent();
    torqueCurrentAmps = talonFX.getTorqueCurrent();
    temperatureCelsius = talonFX.getDeviceTemp();

    // Initialize sensor inputs
    redundantSwitchOne = new DigitalInput(1);
    redundantSwitchTwo = new DigitalInput(2);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        positionRotations,
        velocityRotationsPerSecond,
        appliedVolts,
        supplyCurrentAmps,
        torqueCurrentAmps,
        temperatureCelsius);
    talonFX.optimizeBusUtilization();

    voltageRequest = new VoltageOut(0.0);

    PhoenixUtil.registerSignals(
        true,
        positionRotations,
        velocityRotationsPerSecond,
        appliedVolts,
        supplyCurrentAmps,
        torqueCurrentAmps,
        temperatureCelsius);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    InternalLoggedTracer.reset();

    // Update in standardized order
    inputs.positionRadians = Units.rotationsToRadians(positionRotations.getValueAsDouble());
    inputs.velocityRadiansPerSecond =
        Units.rotationsToRadians(velocityRotationsPerSecond.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();

    // Update sensor inputs
    inputs.redundantSwitchOne = redundantSwitchOne.get();
    inputs.redundantSwitchTwo = redundantSwitchTwo.get();

    InternalLoggedTracer.record("Refresh Update Inputs", "Climber/TalonFX");
  }

  @Override
  public void setVoltage(double volts) {
    InternalLoggedTracer.reset();
    talonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
    InternalLoggedTracer.record("Set Voltage", "Climber/TalonFX");
  }

  @Override
  public boolean isClimbed() {
    return positionRotations.getValueAsDouble()
        >= Units.radiansToRotations(ClimberConstants.CLIMBER_CLIMBED_RADIANS);
  }
}
