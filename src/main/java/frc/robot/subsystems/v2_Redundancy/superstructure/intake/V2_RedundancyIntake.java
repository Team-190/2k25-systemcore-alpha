// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.v2_Redundancy.superstructure.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructure;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureStates;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeExtensionState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeRollerState;
import frc.robot.util.ExternalLoggedTracer;
import frc.robot.util.InternalLoggedTracer;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V2_RedundancyIntake {
  private final V2_RedundancyIntakeIO io;
  private final V2_RedundancyIntakeIOInputsAutoLogged inputs;

  @Getter
  @AutoLogOutput(key = "Intake/Extension Goal")
  private IntakeExtensionState extensionGoal;

  @Getter
  @AutoLogOutput(key = "Intake/Roller Goal")
  private IntakeRollerState intakeRollerGoal;

  private boolean isClosedLoop;

  public V2_RedundancyIntake(V2_RedundancyIntakeIO io) {
    this.io = io;
    inputs = new V2_RedundancyIntakeIOInputsAutoLogged();

    extensionGoal = IntakeExtensionState.STOW;
    intakeRollerGoal = IntakeRollerState.STOP;

    isClosedLoop = true;
  }

  public void periodic() {
    ExternalLoggedTracer.reset();
    InternalLoggedTracer.reset();
    io.updateInputs(inputs);
    InternalLoggedTracer.record("Update Inputs", "Intake/Periodic");

    InternalLoggedTracer.reset();
    Logger.processInputs("Intake", inputs);
    InternalLoggedTracer.record("Process Inputs", "Intake/Periodic");

    InternalLoggedTracer.reset();
    if (isClosedLoop) {
      io.setExtensionGoal(extensionGoal.getDistance());
    }
    io.setRollerVoltage(intakeRollerGoal.getVoltage());
    InternalLoggedTracer.record("Set Extension Goal", "Intake/Periodic");
    ExternalLoggedTracer.record("Intake Total", "Intake/Periodic");
  }

  public double getExtension() {
    return inputs.extensionPositionMeters;
  }

  /**
   * Sets the goal state of the extension.
   *
   * @param goal The desired IntakeState.
   * @return A command to set the extension goal.
   */
  public void setExtensionGoal(IntakeExtensionState goal) {
    isClosedLoop = true;
    this.extensionGoal = goal;
  }

  /**
   * Sets the voltage of the roller.
   *
   * @param volts The desired voltage.
   * @return A command to set the roller voltage.
   */
  public void setRollerGoal(IntakeRollerState state) {
    this.intakeRollerGoal = state;
  }

  /**
   * Runs the SysId routine for the extension.
   *
   * @return A command to run the SysId routine.
   */
  public Command sysIdRoutine(V2_RedundancySuperstructure superstructure) {
    SysIdRoutine characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(8),
                (state) -> Logger.recordOutput("Intake/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setExtensionVoltage(volts.in(Volts)), null, superstructure));
    return Commands.sequence(
        superstructure.runGoal(V2_RedundancySuperstructureStates.OVERRIDE),
        Commands.runOnce(() -> isClosedLoop = false),
        characterizationRoutine
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    Math.abs(
                            inputs.extensionPositionMeters
                                - V2_RedundancyIntakeConstants.EXTENSION_PARAMS.MAX_EXTENSION())
                        <= V2_RedundancyIntakeConstants.EXTENSION_MOTOR_CONSTRAINTS
                            .GOAL_TOLERANCE()
                            .get()),
        Commands.wait(0.25),
        characterizationRoutine
            .quasistatic(Direction.kReverse)
            .until(
                () ->
                    Math.abs(
                            inputs.extensionPositionMeters
                                - V2_RedundancyIntakeConstants.EXTENSION_PARAMS.MIN_EXTENSION())
                        <= V2_RedundancyIntakeConstants.EXTENSION_MOTOR_CONSTRAINTS
                            .GOAL_TOLERANCE()
                            .get()),
        Commands.wait(0.25),
        characterizationRoutine
            .dynamic(Direction.kForward)
            .until(
                () ->
                    Math.abs(
                            inputs.extensionPositionMeters
                                - V2_RedundancyIntakeConstants.EXTENSION_PARAMS.MAX_EXTENSION())
                        <= V2_RedundancyIntakeConstants.EXTENSION_MOTOR_CONSTRAINTS
                            .GOAL_TOLERANCE()
                            .get()),
        Commands.wait(0.25),
        characterizationRoutine
            .dynamic(Direction.kReverse)
            .until(
                () ->
                    Math.abs(
                            inputs.extensionPositionMeters
                                - V2_RedundancyIntakeConstants.EXTENSION_PARAMS.MIN_EXTENSION())
                        <= V2_RedundancyIntakeConstants.EXTENSION_MOTOR_CONSTRAINTS
                            .GOAL_TOLERANCE()
                            .get()));
  }

  public boolean hasCoral() {
    return Math.abs(inputs.extensionTorqueCurrentAmps) >= 30.0;
  }

  /**
   * Checks if the extension motor is at the goal position.
   *
   * @return True if the extension motor is at the goal, false otherwise.
   */
  @AutoLogOutput(key = "Intake/At Goal")
  public boolean atGoal() {
    return io.atExtensionPositionGoal();
  }

  public double getDistance() {
    return inputs.extensionPositionMeters;
  }

  /**
   * Updates the PID gains for the extension.
   *
   * @param kP The proportional gain.
   * @param kD The derivative gain.
   * @param kS The static gain.
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   */
  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    io.updateGains(kP, kD, kS, kV, kA);
  }

  /**
   * Updates the motion constraints for the extension.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param maxVelocity The maximum velocity.
   */
  public void updateConstraints(double maxAcceleration, double maxVelocity) {
    io.updateConstraints(maxAcceleration, maxVelocity);
  }

  public Command setIntakeVoltage(double volts) {
    return Commands.run(() -> io.setExtensionVoltage(volts));
  }

  public Command homingSequence() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              isClosedLoop = false;
              io.maxExt();
            }),
        setIntakeVoltage(-6).until(() -> Math.abs(inputs.extensionTorqueCurrentAmps) > 45),
        setIntakeVoltage(0),
        Commands.runOnce(() -> io.resetExtension()));
  }

  public Command waitUntilExtensionAtGoal() {
    return Commands.sequence(Commands.wait(0.02), Commands.waitUntil(this::atGoal));
  }
}
