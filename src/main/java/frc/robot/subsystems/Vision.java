package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.Utils;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.TagUtils;

/**
 * Vision subsystem handles pose estimation using PhotonVision and AprilTags.
 * 
 * This subsystem uses two PhotonVision cameras to estimate the robot's pose on the field
 * by detecting AprilTags. The pose estimates are fused with the drivetrain's odometry
 * to maintain accurate position tracking even when wheels slip or vision is temporarily unavailable.
 * 
 * Features:
 * - Dual camera setup (left and right) for better coverage
 * - Multi-tag pose estimation for improved accuracy
 * - Standard deviation scaling based on number of tags and distance
 * - Ambiguity gating to reject low-confidence single-tag detections
 * - Distance-based uncertainty scaling for far tags
 */
public class Vision extends SubsystemBase {

    // Camera instances
    /** Right camera (EagleEye01) */
    private final PhotonCamera camera1 = new PhotonCamera(CAMERA_NAME_1);
    /** Left camera (EagleEye02) */
    private final PhotonCamera camera2 = new PhotonCamera(CAMERA_NAME_2);

    // Pose estimators - convert camera detections to robot poses
    /** Pose estimator for camera1 */
    private final PhotonPoseEstimator estimator1;
    /** Pose estimator for camera2 */
    private final PhotonPoseEstimator estimator2;

    // Telemetry state - latest vision measurement data
    /** Most recent pose estimate from vision */
    private Pose2d latestPose;
    /** Standard deviations for the latest pose estimate (uncertainty) */
    private Matrix<N3, N1> latestStdDevs;
    /** Timestamp of the latest vision measurement */
    private double latestTimestamp = -1.0;
    /** Name of camera that provided latest measurement */
    private String latestCameraName = "";
    /** ID of the most recently seen AprilTag */
    private int lastSeenTagId = -1;
    /** Ambiguity value of latest detection (lower is better) */
    private double latestAmbiguity = 1.0;
    /** Number of tags detected in latest measurement */
    private int latestNumTags = 0;

    public Vision() {
        camera1.setPipelineIndex(0);
        camera2.setPipelineIndex(0);

        estimator1 = new PhotonPoseEstimator(
                TAG_LAYOUT,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                ROBOT_TO_CAM_1);

        estimator2 = new PhotonPoseEstimator(
                TAG_LAYOUT,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                ROBOT_TO_CAM_2);

        estimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        estimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        processCamera(camera1, estimator1);
        processCamera(camera2, estimator2);

        // Dashboard publishing
        SmartDashboard.putBoolean("Vision/HasPose", latestPose != null);
        SmartDashboard.putNumber("Vision/LastTimestamp", latestTimestamp);
        SmartDashboard.putString("Vision/Camera", latestCameraName);
        SmartDashboard.putNumber("Vision/LastTagId", lastSeenTagId);
        SmartDashboard.putNumber("Vision/LastAmbiguity", latestAmbiguity);
        SmartDashboard.putNumber("Vision/LastNumTags", latestNumTags);

        if (latestStdDevs != null) {
            SmartDashboard.putNumber("Vision/StdDevX", latestStdDevs.get(0, 0));
            SmartDashboard.putNumber("Vision/StdDevY", latestStdDevs.get(1, 0));
            SmartDashboard.putNumber("Vision/StdDevTheta", latestStdDevs.get(2, 0));
        }
    }

    /**
     * Processes vision data from a single camera and fuses it with odometry.
     * 
     * This method:
     * 1. Gets latest camera results
     * 2. Estimates robot pose from AprilTag detections
     * 3. Calculates uncertainty (standard deviations) based on number of tags and distance
     * 4. Applies gating logic (ambiguity, distance) to reject bad measurements
     * 5. Fuses valid measurements with drivetrain odometry
     * 
     * @param camera PhotonVision camera to process
     * @param estimator Pose estimator for this camera
     */
    private void processCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
        // Get all unread results from camera
        var results = camera.getAllUnreadResults();
        if (results.isEmpty()) return;

        // Use most recent result
        var last = results.get(results.size() - 1);
        var maybeEst = estimator.update(last);
        if (maybeEst.isEmpty()) return;

        // Extract pose estimate
        var visionEst = maybeEst.get();
        Pose2d visionPose = visionEst.estimatedPose.toPose2d();
        var targets = last.getTargets();

        if (targets.isEmpty()) return;

        // Calculate number of tags and average distance for uncertainty scaling
        int numTags = 0;
        double avgTagDist = 0.0;

        for (var tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;

            lastSeenTagId = tgt.getFiducialId();
            numTags++;
            // Calculate distance from robot to tag
            avgTagDist += tagPose.get().toPose2d()
                    .getTranslation()
                    .getDistance(visionPose.getTranslation());
        }

        if (numTags == 0) return;

        avgTagDist /= numTags;

        // Start with base standard deviations (multi-tag is more accurate)
        Matrix<N3, N1> stdDevs = (numTags > 1)
                ? MULTI_TAG_STD_DEVS  // Multiple tags = lower uncertainty
                : SINGLE_TAG_STD_DEVS; // Single tag = higher uncertainty

        // Ambiguity gating - reject ambiguous single-tag detections
        // Ambiguity > 0.25 means the tag could be at multiple possible poses
        var bestTarget = last.getBestTarget();
        double ambiguity = bestTarget.getPoseAmbiguity();
        if (numTags == 1 && ambiguity > 0.25) return;

        // Distance-based scaling - far tags are less reliable
        if (numTags == 1 && avgTagDist > 3.0) {
            // Tag is very far (>3m), set uncertainty to max to effectively ignore it
            stdDevs = VecBuilder.fill(
                    Double.MAX_VALUE,
                    Double.MAX_VALUE,
                    Double.MAX_VALUE);
        } else {
            // Scale uncertainty based on distance (quadratic scaling)
            double normDist = avgTagDist / 1.5;
            stdDevs = stdDevs.times(1.0 + normDist * normDist);
        }

        // Convert timestamp to current time
        double ts = Utils.fpgaToCurrentTime(visionEst.timestampSeconds);

        // Fuse vision measurement with drivetrain odometry
        // This is the critical operation that updates the robot's pose estimate
        RobotContainer.m_drivetrain.addVisionMeasurement(
                visionPose,
                ts,
                stdDevs
        );

        // Store telemetry for debugging/monitoring
        latestPose = visionPose;
        latestStdDevs = stdDevs;
        latestTimestamp = ts;
        latestCameraName = camera.getName();
        latestAmbiguity = ambiguity;
        latestNumTags = numTags;
    }

    public int getLastSeenTagId() {
        return lastSeenTagId;
    }

    public int getClosestTagId(Pose2d robotPose) {
        List<Integer> allTags = Constants.Vision.TAGS;
        return allTags.stream()
                .min(Comparator.comparingDouble(id ->
                        TagUtils.getTagPose2d(id)
                                .map(tagPose -> tagPose.getTranslation()
                                        .getDistance(robotPose.getTranslation()))
                                .orElse(Double.MAX_VALUE)))
                .orElse(Constants.Vision.TAGS.get(0));
    }
}
