// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.visionlimelight.Camera;
import frc.robot.subsystems.shared.visionlimelight.CameraDuty;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ExternalLoggedTracer;
import frc.robot.util.GeometryUtil;
import frc.robot.util.InternalLoggedTracer;
import frc.robot.util.NTPrefixes;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static Rotation2d robotHeading;
  private static Rotation2d headingOffset;
  private static SwerveModulePosition[] modulePositions;

  private static final SwerveDrivePoseEstimator fieldLocalizer;
  private static final SwerveDrivePoseEstimator reefLocalizer;
  private static final SwerveDriveOdometry odometry;

  @Getter private static ReefAlignData reefAlignData;
  @Getter private static OperatorInputData OIData;
  @Getter private static RobotConfigurationData robotConfigurationData;

  @Getter @Setter private static RobotMode mode;
  @Getter @Setter private static boolean hasAlgae;

  @Getter @Setter private static boolean isIntakingCoral;
  @Getter @Setter private static boolean isAutoAligning;
  @Getter @Setter private static boolean autoClapOverride;

  static {
    switch (Constants.ROBOT) {
      case V2_REDUNDANCY:
      case V2_REDUNDANCY_SIM:
        break;
    }

    OIData = new OperatorInputData(ReefPose.LEFT, ReefState.STOW);

    robotHeading = new Rotation2d();
    headingOffset = new Rotation2d();
    modulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i] = new SwerveModulePosition();
    }

    fieldLocalizer =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_CONFIG.kinematics(),
            new Rotation2d(),
            modulePositions,
            new Pose2d());
    reefLocalizer =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_CONFIG.kinematics(),
            new Rotation2d(),
            modulePositions,
            new Pose2d());
    odometry =
        new SwerveDriveOdometry(
            DriveConstants.DRIVE_CONFIG.kinematics(), new Rotation2d(), modulePositions);

    reefAlignData =
        new ReefAlignData(
            -1, new Pose2d(), new Pose2d(), 0.0, 0.0, false, false, ReefState.ALGAE_INTAKE_BOTTOM);
    robotConfigurationData = new RobotConfigurationData(0.0, new Rotation2d(), 0.0);
  }

  public RobotState() {}

  public static void periodic(
      Rotation2d robotHeading,
      long latestRobotHeadingTimestamp,
      double robotYawVelocity,
      Translation2d robotFieldRelativeVelocity,
      SwerveModulePosition[] modulePositions,
      Camera[] cameras) {
    ExternalLoggedTracer.reset();
    RobotState.robotHeading = robotHeading;
    RobotState.modulePositions = modulePositions;

    fieldLocalizer.updateWithTime(Timer.getTimestamp(), robotHeading, modulePositions);
    reefLocalizer.updateWithTime(Timer.getTimestamp(), robotHeading, modulePositions);
    odometry.update(robotHeading, modulePositions);

    for (Camera camera : cameras) {
      double[] limelightHeadingData = {
        robotHeading.minus(headingOffset).getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0
      };
      camera.getRobotHeadingPublisher().set(limelightHeadingData, latestRobotHeadingTimestamp);
    }

    NetworkTableInstance.getDefault().flush();

    for (Camera camera : cameras) {
      if (camera.getCameraDuties().contains(CameraDuty.FIELD_LOCALIZATION)
          && camera.getTargetAquired()
          && !GeometryUtil.isZero(camera.getPrimaryPose())
          && !GeometryUtil.isZero(camera.getSecondaryPose())
          && Math.abs(robotYawVelocity) <= Units.degreesToRadians(15.0)
          && Math.abs(robotFieldRelativeVelocity.getNorm()) <= 1.0
          && camera.getTotalTargets() > 0) {
        double xyStddevPrimary =
            camera.getPrimaryXYStandardDeviationCoefficient()
                * Math.pow(camera.getAverageDistance(), 2.0)
                / camera.getTotalTargets()
                * camera.getHorizontalFOV();
        fieldLocalizer.addVisionMeasurement(
            camera.getPrimaryPose(),
            camera.getFrameTimestamp(),
            VecBuilder.fill(xyStddevPrimary, xyStddevPrimary, Double.POSITIVE_INFINITY));
        if (camera.getTotalTargets() > 1) {
          double xyStddevSecondary =
              camera.getSecondaryXYStandardDeviationCoefficient()
                  * Math.pow(camera.getAverageDistance(), 2.0)
                  / camera.getTotalTargets()
                  * camera.getHorizontalFOV();
          fieldLocalizer.addVisionMeasurement(
              camera.getSecondaryPose(),
              camera.getFrameTimestamp(),
              VecBuilder.fill(xyStddevSecondary, xyStddevSecondary, Double.POSITIVE_INFINITY));
        }
      }
    }

    int closestReefTag = getMinDistanceReefTag();

    for (Camera camera : cameras) {
      if (camera.getCameraDuties().contains(CameraDuty.REEF_LOCALIZATION)
          && !GeometryUtil.isZero(camera.getPrimaryPose())
          && camera.getTotalTargets() > 0) {
        double xyStddevPrimary =
            camera.getPrimaryXYStandardDeviationCoefficient()
                * Math.pow(camera.getAverageDistance(), 2.0)
                / camera.getTotalTargets()
                * camera.getHorizontalFOV();
        reefLocalizer.addVisionMeasurement(
            camera.getPrimaryPose(),
            camera.getFrameTimestamp(),
            VecBuilder.fill(xyStddevPrimary, xyStddevPrimary, Double.POSITIVE_INFINITY));
      }
    }

    Pose2d autoAlignCoralSetpoint =
        OIData.currentReefHeight().equals(ReefState.L1)
            ? Reef.reefMap.get(closestReefTag).getPostSetpoint(ReefPose.CENTER)
            : Reef.reefMap.get(closestReefTag).getPostSetpoint(OIData.currentReefPost());
    Pose2d autoAlignAlgaeSetpoint =
        Reef.reefMap.get(getMinDistanceReefAlgaeTag()).getAlgaeSetpoint();

    double distanceToCoralSetpoint =
        RobotState.getRobotPoseReef()
            .getTranslation()
            .getDistance(autoAlignCoralSetpoint.getTranslation());
    double distanceToAlgaeSetpoint =
        RobotState.getRobotPoseReef()
            .getTranslation()
            .getDistance(autoAlignAlgaeSetpoint.getTranslation());

    boolean atCoralSetpoint =
        Math.abs(distanceToCoralSetpoint)
            <= DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.positionThresholdMeters().get();
    boolean atAlgaeSetpoint =
        Math.abs(distanceToAlgaeSetpoint)
            <= DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.positionThresholdMeters().get();

    ReefState algaeHeight;
    switch (closestReefTag) {
      case 10, 6, 8, 21, 17, 19:
        algaeHeight = ReefState.ALGAE_INTAKE_BOTTOM;
        break;
      case 9, 11, 7, 22, 20, 18:
        algaeHeight = ReefState.ALGAE_INTAKE_TOP;
        break;
      default:
        algaeHeight = ReefState.ALGAE_INTAKE_BOTTOM;
        break;
    }
    ;

    reefAlignData =
        new ReefAlignData(
            closestReefTag,
            autoAlignCoralSetpoint,
            autoAlignAlgaeSetpoint,
            distanceToCoralSetpoint,
            distanceToAlgaeSetpoint,
            atCoralSetpoint,
            atAlgaeSetpoint,
            algaeHeight,
            cameras);

    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Has Algae", hasAlgae);

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Field Pose", fieldLocalizer.getEstimatedPosition());
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Odometry Pose", odometry.getPose());
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Heading Offset", headingOffset);

    Logger.recordOutput(NTPrefixes.OI_DATA + "Reef Post", OIData.currentReefPost());
    Logger.recordOutput(NTPrefixes.OI_DATA + "Reef Height", OIData.currentReefHeight());

    Logger.recordOutput(NTPrefixes.REEF_DATA + "Reef Pose", reefLocalizer.getEstimatedPosition());
    Logger.recordOutput(NTPrefixes.REEF_DATA + "Closest Reef April Tag", closestReefTag);

    Logger.recordOutput(NTPrefixes.CORAL_DATA + "Coral Setpoint", autoAlignCoralSetpoint);
    Logger.recordOutput(NTPrefixes.CORAL_DATA + "Coral Setpoint Error", distanceToCoralSetpoint);
    Logger.recordOutput(NTPrefixes.CORAL_DATA + "At Coral Setpoint", atCoralSetpoint);

    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "Algae Setpoint", autoAlignAlgaeSetpoint);
    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "Algae Setpoint Error", distanceToAlgaeSetpoint);
    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "At Algae Setpoint", atAlgaeSetpoint);
    ExternalLoggedTracer.record("Robot State Total", "RobotState/Periodic");
  }

  public static void periodic(
      Rotation2d robotHeading,
      long latestRobotHeadingTimestamp,
      double robotYawVelocity,
      Translation2d robotFieldRelativeVelocity,
      SwerveModulePosition[] modulePositions,
      double intakeStart,
      Rotation2d armStart,
      double elevatorStart,
      Camera[] cameras) {
    ExternalLoggedTracer.reset();
    InternalLoggedTracer.reset();
    RobotState.robotHeading = robotHeading;
    RobotState.modulePositions = modulePositions;
    InternalLoggedTracer.record("Reset Instance Variables", "RobotState/Periodic");

    InternalLoggedTracer.reset();
    fieldLocalizer.updateWithTime(Timer.getTimestamp(), robotHeading, modulePositions);
    reefLocalizer.updateWithTime(Timer.getTimestamp(), robotHeading, modulePositions);
    odometry.update(robotHeading, modulePositions);
    InternalLoggedTracer.record("Update Localizers", "RobotState/Periodic");

    InternalLoggedTracer.reset();
    for (Camera camera : cameras) {
      double[] limelightHeadingData = {
        robotHeading.minus(headingOffset).getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0
      };
      camera.getRobotHeadingPublisher().set(limelightHeadingData, latestRobotHeadingTimestamp);
    }
    InternalLoggedTracer.record("Publish Heading To LLs", "RobotState/Periodic");

    InternalLoggedTracer.reset();
    NetworkTableInstance.getDefault().flush();
    InternalLoggedTracer.record("Flush NT", "RobotState/Periodic");

    InternalLoggedTracer.reset();
    for (Camera camera : cameras) {
      if (camera.getCameraDuties().contains(CameraDuty.FIELD_LOCALIZATION)
          && camera.getTargetAquired()
          && !GeometryUtil.isZero(camera.getPrimaryPose())
          && Math.abs(robotYawVelocity) <= Units.degreesToRadians(15.0)
          && Math.abs(robotFieldRelativeVelocity.getNorm()) <= 1.0
          && camera.getTotalTargets() > 0
          && RobotMode.enabled()) {
        double xyStddevPrimary =
            camera.getPrimaryXYStandardDeviationCoefficient()
                * Math.pow(camera.getAverageDistance(), 2.0)
                / camera.getTotalTargets()
                * camera.getHorizontalFOV();
        fieldLocalizer.addVisionMeasurement(
            camera.getPrimaryPose(),
            camera.getFrameTimestamp(),
            VecBuilder.fill(xyStddevPrimary, xyStddevPrimary, Double.POSITIVE_INFINITY));
      }
      if (camera.getCameraDuties().contains(CameraDuty.FIELD_LOCALIZATION)
          && camera.getTargetAquired()
          && !GeometryUtil.isZero(camera.getSecondaryPose())
          && Math.abs(robotYawVelocity) <= Units.degreesToRadians(10.0)
          && Math.abs(robotFieldRelativeVelocity.getNorm()) <= 1.0
          && camera.getTotalTargets() > 1
          && RobotMode.disabled()) {
        double xyStddevSecondary =
            camera.getSecondaryXYStandardDeviationCoefficient()
                * Math.pow(camera.getAverageDistance(), 2.0)
                / camera.getTotalTargets()
                * camera.getHorizontalFOV();
        fieldLocalizer.addVisionMeasurement(
            camera.getSecondaryPose(),
            camera.getFrameTimestamp(),
            VecBuilder.fill(xyStddevSecondary, xyStddevSecondary, 0.1));
      }
    }
    InternalLoggedTracer.record("Add Field Localizer Measurements", "RobotState/Periodic");

    InternalLoggedTracer.reset();
    int closestReefTag = getMinDistanceReefTag();
    InternalLoggedTracer.record("Get Minimum Distance To Reef Tag", "RobotState/Periodic");

    InternalLoggedTracer.reset();
    for (Camera camera : cameras) {
      if (camera.getCameraDuties().contains(CameraDuty.REEF_LOCALIZATION)
          && !GeometryUtil.isZero(camera.getPrimaryPose())
          && camera.getTotalTargets() > 0) {
        double xyStddevPrimary =
            camera.getPrimaryXYStandardDeviationCoefficient()
                * Math.pow(camera.getAverageDistance(), 2.0)
                / camera.getTotalTargets()
                * camera.getHorizontalFOV();
        reefLocalizer.addVisionMeasurement(
            camera.getPrimaryPose(),
            camera.getFrameTimestamp(),
            VecBuilder.fill(xyStddevPrimary, xyStddevPrimary, Double.POSITIVE_INFINITY));
      }
    }

    if (RobotMode.disabled()) {
      resetRobotPose(getRobotPoseField());
    }

    InternalLoggedTracer.record("Add Reef Localizer Measurements", "RobotState/Periodic");

    InternalLoggedTracer.reset();
    Pose2d autoAlignCoralSetpoint =
        OIData.currentReefHeight().equals(ReefState.L1)
            ? Reef.reefMap.get(closestReefTag).getPostSetpoint(ReefPose.CENTER)
            : Reef.reefMap.get(closestReefTag).getPostSetpoint(OIData.currentReefPost());
    Pose2d autoAlignAlgaeSetpoint = Reef.reefMap.get(closestReefTag).getAlgaeSetpoint();

    double distanceToCoralSetpoint =
        RobotState.getRobotPoseReef()
            .getTranslation()
            .getDistance(autoAlignCoralSetpoint.getTranslation());
    double distanceToAlgaeSetpoint =
        RobotState.getRobotPoseReef()
            .getTranslation()
            .getDistance(autoAlignAlgaeSetpoint.getTranslation());

    boolean atCoralSetpoint =
        Math.abs(distanceToCoralSetpoint)
            <= DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.positionThresholdMeters().get();
    boolean atAlgaeSetpoint =
        Math.abs(distanceToAlgaeSetpoint)
            <= DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.positionThresholdMeters().get();

    ReefState algaeHeight;
    switch (closestReefTag) {
      case 10:
      case 6:
      case 8:
      case 21:
      case 17:
      case 19:
        algaeHeight = ReefState.ALGAE_INTAKE_BOTTOM;
        break;
      case 9:
      case 11:
      case 7:
      case 22:
      case 20:
      case 18:
        algaeHeight = ReefState.ALGAE_INTAKE_TOP;
        break;
      default:
        algaeHeight = ReefState.ALGAE_INTAKE_BOTTOM;
        break;
    }
    InternalLoggedTracer.record("Generate Setpoints", "RobotState/Periodic");

    InternalLoggedTracer.reset();
    reefAlignData =
        new ReefAlignData(
            closestReefTag,
            autoAlignCoralSetpoint,
            autoAlignAlgaeSetpoint,
            distanceToCoralSetpoint,
            distanceToAlgaeSetpoint,
            atCoralSetpoint,
            atAlgaeSetpoint,
            algaeHeight,
            cameras);

    robotConfigurationData = new RobotConfigurationData(intakeStart, armStart, elevatorStart);
    InternalLoggedTracer.record("Generate Records", "RobotState/Periodic");

    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Has Algae", hasAlgae);

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Field Pose", fieldLocalizer.getEstimatedPosition());
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Odometry Pose", odometry.getPose());
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Heading Offset", headingOffset);

    Logger.recordOutput(NTPrefixes.OI_DATA + "Reef Post", OIData.currentReefPost());
    Logger.recordOutput(NTPrefixes.OI_DATA + "Reef Height", OIData.currentReefHeight());

    Logger.recordOutput(NTPrefixes.REEF_DATA + "Reef Pose", reefLocalizer.getEstimatedPosition());
    Logger.recordOutput(NTPrefixes.REEF_DATA + "Closest Reef April Tag", closestReefTag);

    Logger.recordOutput(NTPrefixes.CORAL_DATA + "Coral Setpoint", autoAlignCoralSetpoint);
    Logger.recordOutput(NTPrefixes.CORAL_DATA + "Coral Setpoint Error", distanceToCoralSetpoint);
    Logger.recordOutput(NTPrefixes.CORAL_DATA + "At Coral Setpoint", atCoralSetpoint);

    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "Algae Setpoint", autoAlignAlgaeSetpoint);
    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "Algae Setpoint Error", distanceToAlgaeSetpoint);
    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "At Algae Setpoint", atAlgaeSetpoint);
    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "Algae Height", algaeHeight);
    ExternalLoggedTracer.record("Robot State Total", "RobotState/Periodic");
  }

  public static Pose2d getRobotPoseField() {
    return fieldLocalizer.getEstimatedPosition();
  }

  public static Pose2d getRobotPoseReef() {
    return reefLocalizer.getEstimatedPosition();
  }

  public static Pose2d getRobotPoseOdometry() {
    return odometry.getPose();
  }

  private static int getMinDistanceReefTag() {
    int minDistanceTag = 1;
    double minDistance = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 6; i++) {
      double currentDistance =
          RobotState.getRobotPoseReef()
              .getTranslation()
              .getDistance(
                  AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[i]).getTranslation());
      if (currentDistance < minDistance) {
        minDistance = currentDistance;
        minDistanceTag = i;
      }
    }

    if (AllianceFlipUtil.shouldFlip()) {
      switch (minDistanceTag) {
        case 0:
          minDistanceTag = 7;
          break;
        case 1:
          minDistanceTag = 6;
          break;
        case 2:
          minDistanceTag = 11;
          break;
        case 3:
          minDistanceTag = 10;
          break;
        case 4:
          minDistanceTag = 9;
          break;
        case 5:
          minDistanceTag = 8;
          break;
      }
    } else {
      switch (minDistanceTag) {
        case 0:
          minDistanceTag = 18;
          break;
        case 1:
          minDistanceTag = 19;
          break;
        case 2:
          minDistanceTag = 20;
          break;
        case 3:
          minDistanceTag = 21;
          break;
        case 4:
          minDistanceTag = 22;
          break;
        case 5:
          minDistanceTag = 17;
          break;
      }
    }
    return minDistanceTag;
  }

  private static int getMinDistanceReefAlgaeTag() {
    int minDistanceTag = 1;
    double minDistance = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 6; i++) {
      double currentDistance =
          RobotState.getRobotPoseReef()
              .getTranslation()
              .getDistance(
                  AllianceFlipUtil.overrideApply(FieldConstants.Reef.centerFaces[i])
                      .getTranslation());
      if (currentDistance < minDistance) {
        minDistance = currentDistance;
        minDistanceTag = i;
      }
    }

    for (int i = 0; i < 6; i++) {
      double currentDistance =
          RobotState.getRobotPoseReef()
              .getTranslation()
              .getDistance((FieldConstants.Reef.centerFaces[i]).getTranslation());
      if (currentDistance < minDistance) {
        minDistance = currentDistance;
        minDistanceTag = i + 6;
      }
    }

    switch (minDistanceTag) {
      case 0:
        minDistanceTag = 7;
        break;
      case 1:
        minDistanceTag = 6;
        break;
      case 2:
        minDistanceTag = 11;
        break;
      case 3:
        minDistanceTag = 10;
        break;
      case 4:
        minDistanceTag = 9;
        break;
      case 5:
        minDistanceTag = 8;
        break;
      case 6:
        minDistanceTag = 18;
        break;
      case 7:
        minDistanceTag = 19;
        break;
      case 8:
        minDistanceTag = 20;
        break;
      case 9:
        minDistanceTag = 21;
        break;
      case 10:
        minDistanceTag = 22;
        break;
      case 11:
        minDistanceTag = 17;
        break;
    }
    return minDistanceTag;
  }

  public static void resetRobotPose(Pose2d pose) {
    System.out.println("Got here");
    headingOffset = robotHeading.minus(pose.getRotation());
    fieldLocalizer.resetPosition(robotHeading, modulePositions, pose);
    reefLocalizer.resetPosition(robotHeading, modulePositions, pose);
    odometry.resetPosition(robotHeading, modulePositions, pose);
  }

  public static void setReefPost(ReefPose post) {
    ReefState height = OIData.currentReefHeight();
    OIData = new OperatorInputData(post, height);
  }

  public static void toggleReefPost() {
    ReefState height = OIData.currentReefHeight();
    if (OIData.currentReefPost().equals(ReefPose.LEFT)) {
      OIData = new OperatorInputData(ReefPose.RIGHT, height);
    } else {
      OIData = new OperatorInputData(ReefPose.LEFT, height);
    }
  }

  public static void setReefHeight(ReefState height) {
    ReefPose post = OIData.currentReefPost();
    OIData = new OperatorInputData(post, height);
  }

  public static final record ReefAlignData(
      int closestReefTag,
      Pose2d coralSetpoint,
      Pose2d algaeSetpoint,
      double distanceToCoralSetpoint,
      double distanceToAlgaeSetpoint,
      boolean atCoralSetpoint,
      boolean atAlgaeSetpoint,
      ReefState algaeIntakeHeight,
      Camera... cameras) {}

  public static final record OperatorInputData(
      ReefPose currentReefPost, ReefState currentReefHeight) {}

  public static final record RobotConfigurationData(
      double intakeStart, Rotation2d armStart, double elevatorStart) {}

  public enum RobotMode {
    DISABLED,
    TELEOP,
    AUTO;

    public static boolean enabled(RobotMode mode) {
      return mode.equals(TELEOP) || mode.equals(AUTO);
    }

    public static boolean disabled(RobotMode mode) {
      return mode.equals(DISABLED);
    }

    public static boolean teleop(RobotMode mode) {
      return mode.equals(TELEOP);
    }

    public static boolean auto(RobotMode mode) {
      return mode.equals(AUTO);
    }

    public static boolean enabled() {
      return enabled(RobotState.getMode());
    }

    public static boolean disabled() {
      return disabled(RobotState.getMode());
    }

    public static boolean teleop() {
      return teleop(RobotState.getMode());
    }

    public static boolean auto() {
      return auto(RobotState.getMode());
    }
  }
}
