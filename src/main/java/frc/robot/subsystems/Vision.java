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

import frc.robot.Constants;
import frc.robot.util.TagUtils;

public class Vision extends SubsystemBase {

    // Cameras
    private final PhotonCamera camera1 = new PhotonCamera(CAMERA_NAME_1);
    private final PhotonCamera camera2 = new PhotonCamera(CAMERA_NAME_2);

    // Pose estimators
    private final PhotonPoseEstimator estimator1;
    private final PhotonPoseEstimator estimator2;

    // Latest fused vision result (from either camera)
    private Pose2d latestPose;
    private Matrix<N3, N1> latestStdDevs;
    private double latestTimestamp = -1.0;
    private String latestCameraName = "";
    private int lastSeenTagId = -1;
    private double latestAmbiguity = 1.0;
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
        // Try to update from both cameras; the internal logic chooses which one wins
        updateCamera(estimator1, camera1);
        updateCamera(estimator2, camera2);
        // Publish to SmartDashboard
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

    private void updateCamera(PhotonPoseEstimator estimator, PhotonCamera camera) {
        var results = camera.getAllUnreadResults();
        if (results.isEmpty()) return;

        var last = results.get(results.size() - 1);
        var maybeEst = estimator.update(last);
        if (maybeEst.isEmpty()) return;

        var visionEst = maybeEst.get();
        Pose2d visionPose = visionEst.estimatedPose.toPose2d();
        var targets = last.getTargets();

        if (targets.isEmpty()) {
            return;
        }

        int numTags = 0;
        double avgTagDist = 0.0;

        for (var tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;

            lastSeenTagId = tgt.getFiducialId();
            numTags++;
            avgTagDist += tagPose.get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(visionPose.getTranslation());
        }

        if (numTags == 0) {
            return;
        }

        avgTagDist /= numTags;

        // Base std dev selection
        Matrix<N3, N1> stdDevs = (numTags > 1) ? MULTI_TAG_STD_DEVS : SINGLE_TAG_STD_DEVS;

        // Ambiguity gating: reject bad single-tag results
        var bestTarget = last.getBestTarget();
        double ambiguity = bestTarget.getPoseAmbiguity();
        if (numTags == 1 && ambiguity > 0.2) {
            // too ambiguous – skip this measurement
            return;
        }

        // Distance-based rejection for far single-tag
        if (numTags == 1 && avgTagDist > 3.0) {
            stdDevs = VecBuilder.fill(
                    Double.MAX_VALUE,
                    Double.MAX_VALUE,
                    Double.MAX_VALUE);
        } else {
            // Quadratic ramp based on distance
            double normDist = avgTagDist / 1.5;
            double rampFactor = 1.0 + normDist * normDist;
            stdDevs = stdDevs.times(rampFactor);
        }

        // PhotonVision timestamp (seconds since robot start, same timebase as WPILib)
        double ts = visionEst.timestampSeconds;

        // Prefer:
        //  - newer measurements
        //  - and, if timestamps are nearly equal, multi-tag over single-tag
        double qualityBoost = (numTags > 1) ? 1e-4 : 0.0;
        double newScore = ts + qualityBoost;
        double oldScore = latestTimestamp + ((latestNumTags > 1) ? 1e-4 : 0.0);

        if (latestPose == null || newScore > oldScore) {
            latestPose = visionPose;
            latestStdDevs = stdDevs;
            latestTimestamp = ts;
            latestCameraName = camera.getName();
            latestAmbiguity = ambiguity;
            latestNumTags = numTags;
        }
    }

    /** Latest fused vision pose (from either camera), or null if none yet. */
    public Pose2d getLatestPose() {
        return latestPose;
    }

    /** Latest vision standard deviations, or null if none yet. */
    public Matrix<N3, N1> getLatestStdDevs() {
        return latestStdDevs;
    }

    /** Timestamp (seconds) of the latest accepted vision measurement. */
    public double getLatestTimestamp() {
        return latestTimestamp;
    }

    /** ID of the last AprilTag seen by any camera. */
    public int getLastSeenTagId() {
        return lastSeenTagId;
    }

    /** Name of the camera that produced the latest accepted pose. */
    public String getLatestCameraName() {
        return latestCameraName;
    }

    /**
     * Compute closest tag ID to the given robot pose using the field layout.
     * This does NOT require cameras to currently see that tag.
     */
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
