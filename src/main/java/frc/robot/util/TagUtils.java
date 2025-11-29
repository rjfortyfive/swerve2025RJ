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
import frc.robot.RobotContainer;

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
      Pose2d tagPose = getTagPose2d(tagId)
          .orElse(RobotContainer.m_drivetrain.getPose());

      TagOffsetConfig cfg = kTagConfigs.get(tagId);
      if (cfg == null) {
        Logger.warn("No tag config for ID {}", tagId);
        continue;
      }
      
      double effectiveFrontOffset = frontOffsetMeters;
      
      Translation2d front = cfg.frontDir.times(effectiveFrontOffset);
      Translation2d leftShift = cfg.leftDir.times(lateralOffsetMeters);
      Translation2d rightShift = cfg.rightDir.times(lateralOffsetMeters);
      Rotation2d heading = tagPose.getRotation();

      Pose2d[] candidates = new Pose2d[] {
          new Pose2d(tagPose.getTranslation().plus(front), heading), // center
          new Pose2d(tagPose.getTranslation().plus(front).plus(leftShift), heading), // center-left
          new Pose2d(tagPose.getTranslation().plus(front).plus(rightShift), heading) // center-right
      };

      for (Pose2d candidate : candidates) {
        double distance = robotPose.getTranslation().getDistance(candidate.getTranslation());
        if (distance < minDistance) {
          minDistance = distance;
          closestPose = candidate;
        }
      }
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

  public static Pose2d computeTagAdjacencyPose(int tagId, tagSide side, double offsetMeters, double frontoffsetMeters) {
    Pose2d tagPose = getTagPose2d(tagId)
        .orElse(RobotContainer.m_drivetrain.getPose());

    TagOffsetConfig cfg = kTagConfigs.get(tagId);
    if (cfg == null) {
      Logger.warn("No valid tag config for ID {}", tagId);
      return tagPose;
    }
    Translation2d dir = (side == tagSide.LEFT) ? cfg.leftDir : cfg.rightDir;

    // apply tag-specific lateral tweaks for tags 

    double lateral = offsetMeters;
    // if (tagId == 18 && side == tagSide.LEFT) {
    //   lateral += 0.1; // move 0.1 m further left for tag 18
    // }
    // if (tagId == 20 && side == tagSide.RIGHT) {
    //   lateral += 0.18; // move 0.18 m further right for tag 20
    // }
    
    Translation2d override = dir.times(lateral);
    Translation2d front = cfg.frontDir.times(frontoffsetMeters);
    return new Pose2d(
        tagPose.getTranslation().plus(override).plus(front),
        tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));

  }
};

// rotate that local offset into field frame