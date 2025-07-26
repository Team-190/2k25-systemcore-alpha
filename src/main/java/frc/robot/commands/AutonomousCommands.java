package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.RobotState;
import frc.robot.commands.CompositeCommands.V2_RedundancyCompositeCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.visionlimelight.Camera;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructure;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureStates;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.util.LoggedChoreo.LoggedAutoRoutine;
import frc.robot.util.LoggedChoreo.LoggedAutoTrajectory;

public class AutonomousCommands {
  public static void loadAutoTrajectories(Drive drive) {

    drive.getAutoFactory().cache().loadTrajectory("A_LEFT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("A_LEFT_PATH2");
    drive.getAutoFactory().cache().loadTrajectory("A_LEFT_PATH3");
    drive.getAutoFactory().cache().loadTrajectory("A_LEFT_PATH4");

    drive.getAutoFactory().cache().loadTrajectory("A_RIGHT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("A_RIGHT_PATH2");
    drive.getAutoFactory().cache().loadTrajectory("A_RIGHT_PATH3");
    drive.getAutoFactory().cache().loadTrajectory("A_RIGHT_PATH4");

    drive.getAutoFactory().cache().loadTrajectory("B_LEFT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("B_LEFT_PATH2");

    drive.getAutoFactory().cache().loadTrajectory("B_RIGHT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("B_RIGHT_PATH2");

    drive.getAutoFactory().cache().loadTrajectory("C_LEFT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("C_LEFT_PATH2");
    drive.getAutoFactory().cache().loadTrajectory("C_LEFT_PATH3");

    drive.getAutoFactory().cache().loadTrajectory("C_RIGHT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("C_RIGHT_PATH2");
    drive.getAutoFactory().cache().loadTrajectory("C_RIGHT_PATH3");

    drive.getAutoFactory().cache().loadTrajectory("D_CENTER_PATH");
  }

  public static final LoggedAutoRoutine autoALeft(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {
    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoALeft");

    LoggedAutoTrajectory path1 = routine.trajectory("A_LEFT_PATH1");
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("A_LEFT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("A_LEFT_PATH3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path4 =
        routine
            .trajectory("A_LEFT_PATH4")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path4.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoALeftNashoba(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {
    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoALeftNashoba");

    LoggedAutoTrajectory path1 = routine.trajectory("A_LEFT_PATH1");
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("A_LEFT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("A_LEFT_PATH_ALT3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path4 =
        routine
            .trajectory("A_LEFT_PATH_ALT4")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path4.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoALeftDAVE(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {
    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoALeftD.A.V.E.");

    LoggedAutoTrajectory path1 = routine.trajectory("A_LEFT_PATH1");
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("A_LEFT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("A_LEFT_PATH3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path4 =
        routine
            .trajectory("A_LEFT_PATH4_ALT_ALT")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path4.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoARight(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoARight");

    LoggedAutoTrajectory path1 = routine.trajectory("A_RIGHT_PATH1");
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("A_RIGHT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("A_RIGHT_PATH3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path4 =
        routine
            .trajectory("A_RIGHT_PATH4")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.25),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.25),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.25),
                Commands.deadline(
                    path4.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoBLeft(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoBLeft");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("B_LEFT_PATH1")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("B_LEFT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    superstructure.runGoal(V2_RedundancySuperstructureStates.L4)),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    superstructure.runGoal(V2_RedundancySuperstructureStates.L4)),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                superstructure.runGoal(V2_RedundancySuperstructureStates.STOW_DOWN)));

    return routine;
  }

  public static final LoggedAutoRoutine autoCLeft(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCLeft");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("C_LEFT_PATH1")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("C_LEFT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("C_LEFT_PATH3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoCLeftPush(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCLeft");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("C_LEFT_PATH1")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("C_LEFT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("C_LEFT_PATH3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runEnd(
                        () -> drive.runVelocity(new ChassisSpeeds(0.0, -1.0, 0.0)),
                        () -> drive.stop())
                    .withTimeout(0.5),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoCRight(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCRight");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("C_RIGHT_PATH1")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("C_RIGHT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("C_RIGHT_PATH3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5)));
    return routine;
  }

  public static final LoggedAutoRoutine autoCRightPush(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCRight");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("C_RIGHT_PATH1")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("C_RIGHT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("C_RIGHT_PATH3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                Commands.runEnd(
                        () -> drive.runVelocity(new ChassisSpeeds(0.0, 1.0, 0.0)),
                        () -> drive.stop())
                    .withTimeout(0.5),
                path1.cmd(),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5)));
    return routine;
  }

  public static final LoggedAutoRoutine autoBRight(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoBRight");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("B_RIGHT_PATH1")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("B_RIGHT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    superstructure.runGoal(V2_RedundancySuperstructureStates.L4)),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    superstructure.runGoal(V2_RedundancySuperstructureStates.L4)),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                superstructure.runGoal(V2_RedundancySuperstructureStates.STOW_DOWN)));

    return routine;
  }

  public static final LoggedAutoRoutine autoDCenter(
      Drive drive, V2_RedundancySuperstructure superstructure, Camera... cameras) {
    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoDCenter");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("D_CENTER_PATH")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    superstructure.runGoal(V2_RedundancySuperstructureStates.L4)),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5)));
    return routine;
  }
}
