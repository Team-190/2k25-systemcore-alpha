// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.v2_Redundancy.superstructure.manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class V2_RedundancyManipulatorIOSim implements V2_RedundancyManipulatorIO {
  private final SingleJointedArmSim armSim;
  private final DCMotorSim rollerSim;

  private double armAppliedVolts;
  private double rollerAppliedVolts;

  private final ProfiledPIDController feedback;
  private ArmFeedforward feedforward;

  private boolean armClosedLoop;

  public V2_RedundancyManipulatorIOSim() {
    armSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MOTOR_CONFIG(),
                0.004,
                V2_RedundancyManipulatorConstants.ARM_PARAMETERS.GEAR_RATIO()),
            V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MOTOR_CONFIG(),
            V2_RedundancyManipulatorConstants.ARM_PARAMETERS.GEAR_RATIO(),
            V2_RedundancyManipulatorConstants.ARM_PARAMETERS.LENGTH_METERS(),
            V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MIN_ANGLE().getRadians(),
            V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MAX_ANGLE().getRadians(),
            true,
            V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MIN_ANGLE().getRadians());
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 3.0),
            DCMotor.getKrakenX60Foc(1));

    armAppliedVolts = 0.0;
    rollerAppliedVolts = 0.0;

    feedback =
        new ProfiledPIDController(
            V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kP().get(),
            0.0,
            V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kD().get(),
            new Constraints(
                V2_RedundancyManipulatorConstants.CONSTRAINTS
                    .CRUISING_VELOCITY_ROTATIONS_PER_SECOND()
                    .get(),
                V2_RedundancyManipulatorConstants.CONSTRAINTS
                    .MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED()
                    .get()));
    feedforward =
        new ArmFeedforward(
            V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kS().get(),
            V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kV().get(),
            V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kA().get());

    armClosedLoop = true;
  }

  @Override
  public void updateInputs(V2_RedundancyManipulatorIOInputs inputs) {
    if (armClosedLoop)
      armAppliedVolts =
          feedback.calculate(armSim.getAngle())
              + feedforward.calculate(
                  feedback.getSetpoint().position, feedback.getSetpoint().velocity);

    armAppliedVolts = MathUtil.clamp(armAppliedVolts, -12.0, 12.0);
    rollerAppliedVolts = MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0);

    armSim.setInputVoltage(armAppliedVolts);
    armSim.update(Constants.LOOP_PERIOD_SECONDS);
    rollerSim.setInputVoltage(rollerAppliedVolts);
    rollerSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.rollerPosition = Rotation2d.fromRadians(rollerSim.getAngularPosition());
    inputs.rollerVelocityRadiansPerSecond = rollerSim.getAngularVelocity();
    inputs.rollerAccelerationRadiansPerSecondSquared = rollerSim.getAngularAcceleration();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerSupplyCurrentAmps = rollerSim.getCurrentDraw();

    inputs.armPosition = Rotation2d.fromRadians(armSim.getAngle());
    inputs.armVelocityRadiansPerSecond = armSim.getVelocity();
    inputs.armAppliedVolts = armAppliedVolts;
    inputs.armSupplyCurrentAmps = armSim.getCurrentDraw();
    inputs.armPositionGoal = Rotation2d.fromRadians(feedback.getGoal().position);
    inputs.armPositionSetpoint = Rotation2d.fromRadians(feedback.getSetpoint().position);
    inputs.armPositionError = Rotation2d.fromRadians(feedback.getPositionError());
  }

  @Override
  public void setArmPositionGoal(Rotation2d position) {
    armClosedLoop = true;
    feedback.setGoal(position.getRadians());
  }

  @Override
  public void setRollerVoltage(double rollerAppliedVolts) {
    this.rollerAppliedVolts = rollerAppliedVolts;
  }

  @Override
  public void setArmVoltage(double armAppliedVolts) {
    armClosedLoop = false;
    this.armAppliedVolts = armAppliedVolts;
  }

  @Override
  public void updateSlot0ArmGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    feedback.setPID(kP, 0, kD);
    feedforward = new ArmFeedforward(kS, kG, kV);
  }

  @Override
  public void updateArmConstraints(
      double maxAccelerationRadiansPerSecondSquared, double cruisingVelocityRadiansPerSecond) {
    feedback.setConstraints(
        new Constraints(cruisingVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared));
  }
}
