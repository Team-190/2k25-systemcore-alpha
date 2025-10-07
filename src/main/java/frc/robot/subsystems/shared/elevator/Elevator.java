// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shared.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.shared.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructure;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureStates;
import frc.robot.util.ExternalLoggedTracer;
import frc.robot.util.InternalLoggedTracer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  private ElevatorPositions position;
  private boolean isClosedLoop;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();

    position = ElevatorPositions.STOW;
    isClosedLoop = true;
  }

  private void periodic() {
    InternalLoggedTracer.reset();
    io.updateInputs(inputs);
    InternalLoggedTracer.record("Update Inputs", "Elevator/Periodic");

    InternalLoggedTracer.reset();
    Logger.processInputs("Elevator", inputs);
    InternalLoggedTracer.record("Process Inputs", "Elevator/Periodic");

    Logger.recordOutput("Elevator/Position", position.name());

    InternalLoggedTracer.reset();
    if (isClosedLoop) {
      io.setPositionGoal(position.getPosition());
    }
    InternalLoggedTracer.record("Set Position Goal", "Elevator/Periodic");
  }

  /**
   * Gets the elevator position enum for a given reef state.
   *
   * @param newPosition The reef state.
   * @return The corresponding elevator position.
   */
  private ElevatorPositions getPosition(ReefState newPosition) {
    return ElevatorConstants.REEF_STATE_ELEVATOR_POSITION_MAP.get(newPosition);
  }

  /**
   * Gets the current position of the elevator.
   *
   * @return The current elevator position.
   */
  private double getPositionMeters(ElevatorIOInputs inputs) {
    return inputs.positionMeters;
  }

  /**
   * Gets the feedforward characterization velocity.
   *
   * @return The feedforward characterization velocity.
   */
  private double getFFCharacterizationVelocity(ElevatorIOInputs inputs) {
    return inputs.velocityMetersPerSecond
        * ElevatorConstants.ELEVATOR_GEAR_RATIO
        / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS);
  }

  /**
   * Sets the control gains for the elevator.
   *
   * @param kP The proportional gain.
   * @param kD The derivative gain.
   * @param kS The static gain.
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param kG The gravity gain.
   */
  private void setGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    io.updateGains(kP, kD, kS, kV, kA, kG);
  }

  /**
   * Sets the motion constraints for the elevator.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param cruisingVelocity The cruising velocity.
   */
  private void setConstraints(double maxAcceleration, double cruisingVelocity) {
    io.updateConstraints(maxAcceleration, cruisingVelocity);
  }

  /**
   * Checks if the elevator is at the goal position within a specified tolerance.
   *
   * @param position The target position in meters.
   * @return true if the current position is within the goal tolerance of the target position, false
   *     otherwise.
   */
  private boolean atGoal(double position) {
    return Math.abs(position - inputs.positionMeters)
        <= ElevatorConstants.CONSTRAINTS.goalToleranceMeters().get();
  }

  /**
   * Checks if the elevator is at the goal position.
   *
   * @return True if the elevator is at the goal position, false otherwise.
   */
  @AutoLogOutput(key = "Elevator/At Goal")
  private boolean atGoal() {
    return Math.abs(inputs.positionGoalMeters - inputs.positionMeters)
        <= ElevatorConstants.CONSTRAINTS.goalToleranceMeters().get();
  }

  /**
   * Waits until the elevator is at the goal position.
   *
   * @return A command that waits until the elevator is at the goal position.
   */
  private Command waitUntilAtGoal() {
    return Commands.wait(0.02).andThen(Commands.waitUntil(() -> atGoal()));
  }

  /**
   * Checks if the elevator is within a fast scoring tolerance of the goal position.
   *
   * @return A BooleanSupplier that returns true if the elevator is within the fast scoring
   *     tolerance, false otherwise.
   */
  private BooleanSupplier inFastScoringTolerance() {
    return () -> Math.abs(inputs.positionMeters - inputs.positionGoalMeters) <= 0.03;
  }

  /**
   * Runs the SysId routine for the elevator subsystem.
   *
   * @param subsystem The subsystem to run the SysId routine on.
   * @return A command that runs the SysId routine.
   */
  private Command sysIdRoutine(Subsystem subsystem) {

    SysIdRoutine characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(6),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Elevator/SysID State", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, subsystem));

    return Commands.sequence(
        characterizationRoutine
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    Elevator.this.atGoal(
                        ElevatorPositions.L4.getPosition() - Units.inchesToMeters(12.0))),
        characterizationRoutine
            .quasistatic(Direction.kReverse)
            .until(
                () ->
                    Elevator.this.atGoal(
                        ElevatorPositions.STOW.getPosition() + Units.inchesToMeters(12.0))),
        characterizationRoutine
            .dynamic(Direction.kForward)
            .until(
                () ->
                    Elevator.this.atGoal(
                        ElevatorPositions.L4.getPosition() - Units.inchesToMeters(12.0))),
        characterizationRoutine
            .dynamic(Direction.kReverse)
            .until(
                () ->
                    Elevator.this.atGoal(
                        ElevatorPositions.STOW.getPosition() + Units.inchesToMeters(12.0))),
        subsystem.runOnce(() -> setPosition(() -> ReefState.STOW)));
  }

  /**
   * Sets the position of the elevator.
   *
   * @return A command that sets the elevator position.
   */
  private void setPosition() {
    setPosition(() -> RobotState.getOIData().currentReefHeight());
  }

  /**
   * Sets the position of the elevator.
   *
   * @param positionRadians The desired elevator position.
   * @return A command that sets the elevator position.
   */
  private void setPosition(Supplier<ReefState> newPosition) {
    isClosedLoop = true;
    Elevator.this.position = Elevator.this.getPosition(newPosition.get());
    io.setPositionGoal(Elevator.this.position.getPosition());
  }

  public class ElevatorCSB extends SubsystemBase {

    @Override
    public void periodic() {
      ExternalLoggedTracer.reset();
      Elevator.this.periodic();
      ExternalLoggedTracer.record("Elevator Total", "Elevator/Periodic");
    }

    public Command setPosition() {
      return this.runOnce(() -> Elevator.this.setPosition());
    }

    public ElevatorPositions getPosition() {
      return Elevator.this.position;
    }

    public Command setPosition(Supplier<ReefState> newPosition) {
      return this.runOnce(() -> Elevator.this.setPosition(newPosition));
    }

    public Command setVoltage(double volts) {
      return this.runEnd(
          () -> {
            isClosedLoop = false;
            io.setVoltage(volts);
          },
          () -> io.setVoltage(0.0));
    }

    public Command resetPosition() {
      return runOnce(() -> Elevator.this.position = ElevatorPositions.STOW)
          .andThen(
              runOnce(
                  () -> io.setPosition(ElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS())));
    }

    public Command sysIdRoutine() {
      return Elevator.this.sysIdRoutine(this);
    }

    public double getPositionMeters() {
      return Elevator.this.getPositionMeters(inputs);
    }

    public double getFFCharacterizationVelocity() {
      return Elevator.this.getFFCharacterizationVelocity(inputs);
    }

    public void setGains(double kP, double kD, double kS, double kV, double kA, double kG) {
      Elevator.this.setGains(kP, kD, kS, kV, kA, kG);
    }

    public void setConstraints(double maxAcceleration, double cruisingVelocity) {
      Elevator.this.setConstraints(maxAcceleration, cruisingVelocity);
    }

    public boolean atGoal() {
      return Elevator.this.atGoal();
    }

    public Command waitUntilAtGoal() {
      return Elevator.this.waitUntilAtGoal();
    }

    public BooleanSupplier inFastScoringTolerance() {
      return Elevator.this.inFastScoringTolerance();
    }
  }

  public class ElevatorFSM {

    public void periodic() {
      ExternalLoggedTracer.reset();
      Elevator.this.periodic();
      ExternalLoggedTracer.record("Elevator Total", "Elevator/Periodic");
    }

    public ElevatorPositions getPosition() {
      return Elevator.this.position;
    }

    public void setPosition() {
      Elevator.this.setPosition();
    }

    public void setPosition(Supplier<ReefState> newPosition) {
      Elevator.this.setPosition(newPosition);
    }

    public Command resetPosition() {
      return Commands.runOnce(() -> Elevator.this.position = ElevatorPositions.STOW)
          .andThen(
              Commands.runOnce(
                  () -> io.setPosition(ElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS())));
    }

    public Command sysIdRoutine(V2_RedundancySuperstructure superstructure) {

      return Commands.sequence(
          superstructure.runGoal(V2_RedundancySuperstructureStates.OVERRIDE),
          Elevator.this.sysIdRoutine(superstructure));
    }

    public double getPositionMeters() {
      return Elevator.this.getPositionMeters(inputs);
    }

    public double getFFCharacterizationVelocity() {
      return Elevator.this.getFFCharacterizationVelocity(inputs);
    }

    public void setGains(double kP, double kD, double kS, double kV, double kA, double kG) {
      Elevator.this.setGains(kP, kD, kS, kV, kA, kG);
    }

    public void setConstraints(double maxAcceleration, double cruisingVelocity) {
      Elevator.this.setConstraints(maxAcceleration, cruisingVelocity);
    }

    public boolean atGoal() {
      return Elevator.this.atGoal();
    }

    public Command waitUntilAtGoal() {
      return Elevator.this.waitUntilAtGoal();
    }

    public BooleanSupplier inFastScoringTolerance() {
      return Elevator.this.inFastScoringTolerance();
    }
  }

  public ElevatorFSM getFSM() {
    return new ElevatorFSM();
  }

  public ElevatorCSB getCSB() {
    return new ElevatorCSB();
  }
}
