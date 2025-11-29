package frc.robot.util;

import java.util.Optional;
import java.util.Map;
import static java.util.Map.entry;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Logger;

public class TagUtils {
  private static class TagOffsetConfig {
    final Translation2d leftDir;
    final Translation2d rightDir;
    final Translation2d frontDir;

    TagOffsetConfig(Translation2d leftDir, Translation2d rightDir, Translation2d frontDir) {
      this.leftDir = leftDir;
      this.rightDir = rightDir;
      this.frontDir = frontDir;
    }
  }

  private static final Map<Integer, TagOffsetConfig> kTagConfigs = Map.ofEntries(
      entry(6, new TagOffsetConfig(
          new Translation2d(-Math.sqrt(3) / 2, -0.5),
          new Translation2d(Math.sqrt(3) / 2, 0.5),
          new Translation2d(1 / Math.sqrt(3), -2 / Math.sqrt(3)))),
          
      entry(7, new TagOffsetConfig(
          new Translation2d(0, -1),
          new Translation2d(0, 1),
          new Translation2d(1.121, 0))),

      entry(8, new TagOffsetConfig(
          new Translation2d(Math.sqrt(3) / 2, -0.5),
          new Translation2d(-Math.sqrt(3) / 2, 0.5),
          new Translation2d(1 / Math.sqrt(3), 2 / Math.sqrt(3)))),

      entry(9, new TagOffsetConfig(
          new Translation2d(Math.sqrt(3) / 2, 0.5),
          new Translation2d(-Math.sqrt(3) / 2, -0.5),
          new Translation2d(-1 / Math.sqrt(3), 2 / Math.sqrt(3)))),

      entry(10, new TagOffsetConfig(
          new Translation2d(0, 1),
          new Translation2d(0, -1),
          new Translation2d(-1.121, 0) 
      )),
      entry(11, new TagOffsetConfig(
          new Translation2d(-Math.sqrt(3) / 2, 0.5),
          new Translation2d(Math.sqrt(3) / 2, -0.5),
          new Translation2d(-1 / Math.sqrt(3), -2 / Math.sqrt(3)))),
      entry(17, new TagOffsetConfig(
          new Translation2d(-Math.sqrt(3) / 2, 0.5),
          new Translation2d(Math.sqrt(3) / 2, -0.5),
          new Translation2d(-1 / Math.sqrt(3), -2 / Math.sqrt(3)))),
      entry(18, new TagOffsetConfig(
          new Translation2d(0, 1),
          new Translation2d(0, -1),
          new Translation2d(-1.121, 0))),
      entry(19, new TagOffsetConfig(
          new Translation2d(Math.sqrt(3) / 2, 0.5),
          new Translation2d(-Math.sqrt(3) / 2, -0.5),
          new Translation2d(-1 / Math.sqrt(3), 2 / Math.sqrt(3)))),
      entry(20, new TagOffsetConfig(
          new Translation2d(Math.sqrt(3) / 2, -0.5),
          new Translation2d(-Math.sqrt(3) / 2, 0.5),
          new Translation2d(1 / Math.sqrt(3), 2 / Math.sqrt(3)))),
      entry(21, new TagOffsetConfig(
          new Translation2d(0, -1),
          new Translation2d(0, 1),
          new Translation2d(1.121, 0))),
      entry(22, new TagOffsetConfig(
          new Translation2d(-Math.sqrt(3) / 2, -0.5),
          new Translation2d(Math.sqrt(3) / 2, 0.5),
          new Translation2d(1 / Math.sqrt(3), -2 / Math.sqrt(3)))));

  // Pull in your field’s AprilTag layout
  private static final AprilTagFieldLayout kTagLayout = Constants.Vision.kTagLayout;

  /** @return 2D pose of that AprilTag, if it exists in the layout */
  public static Optional<Pose2d> getTagPose2d(int tagId) {
    Logger.warn("Tag Id {}", tagId);
    return kTagLayout
        .getTagPose(tagId)
        .map(p3 -> new Pose2d(p3.getX(), p3.getY(), p3.getRotation().toRotation2d()));
  }

  public static Pose2d getClosestStationPose(
    List<Integer> tagIds,
    Pose2d robotPose,
    double frontOffsetMeters,
    double lateralOffsetMeters) {

  Pose2d closestPose = null;
  double minDistance = Double.MAX_VALUE;

  for (int tagId : tagIds) {

    // Load tag pose safely
    Optional<Pose2d> maybeTagPose = getTagPose2d(tagId);
    if (maybeTagPose.isEmpty()) {
      Logger.warn("Tag {} is missing from field layout", tagId);
      continue; // Skip this tag completely
    }
    Pose2d tagPose = maybeTagPose.get();

    // Load tag offset config safely
    TagOffsetConfig cfg = kTagConfigs.get(tagId);
    if (cfg == null) {
      Logger.warn("No TagOffsetConfig for tag {}", tagId);
      continue;
    }

    // Precompute offsets
    Translation2d front = cfg.frontDir.times(frontOffsetMeters);
    Translation2d leftShift = cfg.leftDir.times(lateralOffsetMeters);
    Translation2d rightShift = cfg.rightDir.times(lateralOffsetMeters);
    Rotation2d heading = tagPose.getRotation();

    // Candidate poses
    Pose2d[] candidates = new Pose2d[] {
        new Pose2d(tagPose.getTranslation().plus(front), heading),              // center
        new Pose2d(tagPose.getTranslation().plus(front).plus(leftShift), heading),  // left
        new Pose2d(tagPose.getTranslation().plus(front).plus(rightShift), heading)  // right
    };

    // Find nearest candidate
    for (Pose2d candidate : candidates) {
      double distance = robotPose.getTranslation().getDistance(candidate.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestPose = candidate;
      }
    }
  }

  // If none valid, return robot pose (or choose what makes sense)
  if (closestPose == null) {
    Logger.warn("No valid station poses found from tag list");
    return robotPose;
  }

  return closestPose;
}

  /**
   * Compute a goal Pose2d offset from the tag.
   * 
   * @param tagId             AprilTag ID
   * @param side              Which side to approach on
   * @param offsetMeters      how far left or right the tag (in meters)
   * @param frontoffsetMeters How far off the reef
   */

   public static Pose2d computeTagAdjacencyPose(
    int tagId,
    tagSide side,
    double offsetMeters,
    double frontoffsetMeters) {

  // Load tag pose safely
  Optional<Pose2d> maybeTagPose = getTagPose2d(tagId);
  if (maybeTagPose.isEmpty()) {
    Logger.warn("Tag {} is missing from field layout", tagId);
    return new Pose2d();
  }
  Pose2d tagPose = maybeTagPose.get();

  // Load config safely
  TagOffsetConfig cfg = kTagConfigs.get(tagId);
  if (cfg == null) {
    Logger.warn("No TagOffsetConfig for tag {}", tagId);
    return tagPose;
  }

  // Lateral translation direction
  Translation2d lateralDir = (side == tagSide.LEFT) ? cfg.leftDir : cfg.rightDir;

  // Offsets
  Translation2d lateral = lateralDir.times(offsetMeters);
  Translation2d front = cfg.frontDir.times(frontoffsetMeters);

  // Final pose
  return new Pose2d(
      tagPose.getTranslation().plus(lateral).plus(front),
      tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));  // Face the tag
}

};

// rotate that local offset into field frame