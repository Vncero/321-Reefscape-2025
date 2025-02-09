/* (C) Robolancers 2025 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.util.MyAlliance;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class ReefAlign {
  /*
    Maps reef AprilTag ("tag") ID to left and right alignment poses,
    loads only the tags on alliance reef as determined at robot initialization
  */
  public static final Map<Integer, Pose2d> leftAlignPoses = new HashMap<>();
  public static final Map<Integer, Pose2d> rightAlignPoses = new HashMap<>();

  private static final Distance kLeftAlignDistance = Inches.of(-6.5);
  private static final Distance kReefDistance = Inches.of(14);
  private static final Distance kRightAlignDistance = Inches.of(6.5);
  private static final Rotation2d kReefAlignmentRotation = Rotation2d.fromDegrees(180);

  private static final List<Integer> blueReefTagIDs = List.of(17, 18, 19, 20, 21, 22);
  private static final List<Integer> redReefTagIDs = List.of(6, 7, 8, 9, 10, 11);

  private static final List<AprilTag> blueReefTags =
      RobotConstants.kAprilTagFieldLayout.getTags().stream()
          .filter(tag -> blueReefTagIDs.contains(tag.ID))
          .toList();
  private static final List<AprilTag> redReefTags =
      RobotConstants.kAprilTagFieldLayout.getTags().stream()
          .filter(tag -> redReefTagIDs.contains(tag.ID))
          .toList();

  /**
   * This method is run during Robot#autonomousInit() and Robot#teleopInit() to save computations,
   * only the poses on the alliance reef are loaded
   */
  public static void loadReefAlignmentPoses() {
    if (MyAlliance.isRed()) {
      for (Integer id : redReefTagIDs) {
        leftAlignPoses.computeIfAbsent(id, ReefAlign::getNearestLeftAlign);
        rightAlignPoses.computeIfAbsent(id, ReefAlign::getNearestRightAlign);
      }
    } else {
      for (Integer id : blueReefTagIDs) {
        leftAlignPoses.computeIfAbsent(id, ReefAlign::getNearestLeftAlign);
        rightAlignPoses.computeIfAbsent(id, ReefAlign::getNearestRightAlign);
      }
    }
  }

  /**
   * Finds the pose of the nearest reef tag on the alliance reef
   *
   * @param robotPose the pose of the robot to find the nearest reef tag pose for
   * @return null if robot alliance is unknown, otherwise a valid reef tag pose
   */
  public static Pose2d getNearestReefPose(Pose2d robotPose) {
    // `Optional` means the Alliance may not exist yet, which must be handled to proceed
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) return null;

    if (alliance.get().equals(Alliance.Blue)) {
      return robotPose.nearest(blueReefTags.stream().map(tag -> tag.pose.toPose2d()).toList());
    } else if (alliance.get().equals(Alliance.Red)) {
      return robotPose.nearest(redReefTags.stream().map(tag -> tag.pose.toPose2d()).toList());
    }

    return null;
  }

  /**
   * Finds the ID of the nearest reef tag on the alliance reef
   *
   * @param robotPose the pose of the robot to find the nearest reef tag ID for
   * @return -1 if robot alliance is unknown, otherwise a valid reef tag ID
   */
  public static int getNearestReefID(Pose2d robotPose) {
    Pose2d nearest = getNearestReefPose(robotPose);

    // handle the null that getNearestReefPose() may return when robot alliance is unknown
    if (nearest == null) return -1;

    return RobotConstants.kAprilTagFieldLayout.getTags().stream()
        .filter(tag -> tag.pose.toPose2d().equals(nearest))
        .toList()
        .get(0)
        .ID;
  }

  /**
   * Intended for aligning to the reef for removing algae
   *
   * @param reefTagID a valid reef tag ID
   * @return null if no reef tag of the ID specified is found, otherwise a valid robot pose aligned
   *     with the center of the nearest reef tag
   */
  private static Pose2d getNearestCenterAlign(int reefTagID) {
    // `Optional` here means there may not be a tag with the specified ID, again must be handled
    Optional<Pose3d> tagPose = RobotConstants.kAprilTagFieldLayout.getTagPose(reefTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose =
        aprilTagPose.plus(new Transform2d(kReefDistance, Meter.zero(), kReefAlignmentRotation));

    return resultPose;
  }

  /**
   * Intended for aligning to the left side reef bars for scoring coral
   *
   * @param reefTagID a valid reef tag ID
   * @return null if no reef tag of the ID specified is found, otherwise a valid robot pose aligned
   *     with the center of the nearest reef tag
   */
  private static Pose2d getNearestLeftAlign(int reefTagID) {
    Optional<Pose3d> tagPose = RobotConstants.kAprilTagFieldLayout.getTagPose(reefTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose =
        aprilTagPose.plus(
            new Transform2d(kReefDistance, kLeftAlignDistance, kReefAlignmentRotation));

    return resultPose;
  }

  /**
   * Intended for aligning to the right side reef bars for scoring coral
   *
   * @param reefTagID a valid reef tag ID
   * @return null if no reef tag of the ID specified is found, otherwise a valid robot pose aligned
   *     with the center of the nearest reef tag
   */
  private static Pose2d getNearestRightAlign(int reefTagID) {
    Optional<Pose3d> tagPose = RobotConstants.kAprilTagFieldLayout.getTagPose(reefTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose =
        aprilTagPose.plus(
            new Transform2d(kReefDistance, kRightAlignDistance, kReefAlignmentRotation));

    return resultPose;
  }

  /** Drives to align against the center of the nearest reef face, no manual driving */
  public static Command goToNearestCenterAlign(SwerveDrive swerveDrive) {
    return swerveDrive.driveToFieldPose(
        () -> getNearestCenterAlign(getNearestReefID(swerveDrive.getPose())));
  }

  /** Drives to align against the left side reef bars of the nearest reef face, no manual driving */
  public static Command goToNearestLeftAlign(SwerveDrive swerveDrive) {
    return swerveDrive.driveToFieldPose(
        () -> getNearestLeftAlign(getNearestReefID(swerveDrive.getPose())));
  }

  /**
   * Drives to align against the right side reef bars of the nearest reef face, no manual driving
   */
  public static Command goToNearestRightAlign(SwerveDrive swerveDrive) {
    return swerveDrive.driveToFieldPose(
        () -> getNearestRightAlign(getNearestReefID(swerveDrive.getPose())));
  }

  /** Maintain translational driving while rotating toward the nearest reef tag */
  public static Command rotateToNearestReefTag(
      SwerveDrive swerveDrive, DoubleSupplier x, DoubleSupplier y) {
    return swerveDrive.driveFixedHeading(
        x,
        y,
        () -> getNearestReefPose(swerveDrive.getPose()).getRotation().plus(Rotation2d.k180deg));
  }
}
