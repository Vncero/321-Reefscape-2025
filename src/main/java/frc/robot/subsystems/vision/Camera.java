/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.RobotConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

@Logged
public class Camera {
  // for logging
  private final String name;

  private final CameraUsage usage;

  private final double
      stdDevMultiplier; // represents relatively how much more or less a camera's estimates are
  // trusted

  private final PhotonCamera camera;

  private final PhotonPoseEstimator poseEstimator;

  private final StructPublisher<Pose3d> estimatedPoseEntry;
  private final StructPublisher<Matrix<N3, N1>> stdDevsEntry;
  private final StructArrayPublisher<Pose3d> targetsSeenEntry;
  // for logging
  private VisionEstimate latestValidEstimate;

  public Camera(CameraConfig config, PhotonCamera camera) {
    this.name = config.cameraName();

    this.usage = config.usage();

    this.stdDevMultiplier = config.relativeStdDevMultiplier();

    this.camera = camera;

    this.poseEstimator =
        new PhotonPoseEstimator(
            RobotConstants.kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.robotToCamera());

    estimatedPoseEntry =
        NetworkTableInstance.getDefault()
            .getStructTopic("/Cameras/" + name + "/EstimatedPose", Pose3d.struct)
            .publish();
    stdDevsEntry =
        NetworkTableInstance.getDefault()
            .getStructTopic("/Cameras/" + name + "/stddevs", Matrix.getStruct(Nat.N3(), Nat.N1()))
            .publish();
    targetsSeenEntry =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/Cameras/" + name + "/TargetsSeen", Pose3d.struct)
            .publish();
  }

  @NotLogged
  public VisionEstimate tryLatestEstimate() {
    if (!camera.isConnected()) return null;

    final var unreadResults = camera.getAllUnreadResults();

    if (unreadResults.isEmpty()) return null;

    final var latestResult = unreadResults.get(unreadResults.size() - 1);

    if (!latestResult.hasTargets()) return null;

    final var estimate = poseEstimator.update(latestResult);

    VisionEstimate est =
        estimate
            .filter(
                poseEst ->
                    VisionConstants.kAllowedFieldArea.contains(
                            poseEst.estimatedPose.getTranslation().toTranslation2d())
                        && poseEst
                            .estimatedPose
                            .getMeasureZ()
                            .isNear(Meters.zero(), VisionConstants.kAllowedFieldHeight))
            .map(
                photonEst -> {
                  final var visionEst =
                      new VisionEstimate(photonEst, calculateStdDevs(photonEst), name, usage);
                  latestValidEstimate = visionEst;
                  return visionEst;
                })
            .orElse(null);

    if (est != null) {
      estimatedPoseEntry.accept(est.estimate().estimatedPose);
      stdDevsEntry.accept(est.stdDevs());
      Pose3d[] targets = new Pose3d[est.estimate().targetsUsed.size()];
      for (int i = 0; i < est.estimate().targetsUsed.size(); i++) {
        targets[i] =
            RobotConstants.kAprilTagFieldLayout
                .getTagPose(est.estimate().targetsUsed.get(i).fiducialId)
                .orElse(Pose3d.kZero);
      }
      targetsSeenEntry.accept(targets);
    } else {
      targetsSeenEntry.accept(new Pose3d[0]);
    }

    return est;
  }

  // could be absolute nonsense, open to tuning constants for each robot camera config
  // assumes `result` has targets
  @NotLogged
  private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose visionPoseEstimate) {
    // weighted average by ambiguity
    final double avgTargetDistance =
        visionPoseEstimate.targetsUsed.stream()
                .mapToDouble(
                    t -> {
                      final double bestCameraToTargetDistance =
                          t.getBestCameraToTarget()
                              .getTranslation()
                              .getDistance(new Translation3d());

                      return bestCameraToTargetDistance;
                    })
                .reduce(0.0, Double::sum)
            / (visionPoseEstimate.targetsUsed.size());

    final double translationStdDev =
        stdDevMultiplier
            * VisionConstants.kTranslationStdDevCoeff
            * Math.pow(avgTargetDistance, 3)
            / Math.pow(visionPoseEstimate.targetsUsed.size(), 3);
    final double rotationStdDev =
        stdDevMultiplier
            * VisionConstants.kRotationStdDevCoeff
            * Math.pow(avgTargetDistance, 3)
            / Math.pow(visionPoseEstimate.targetsUsed.size(), 3);

    return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
  }

  public boolean isConnected() {
    return camera.isConnected();
  }
}
