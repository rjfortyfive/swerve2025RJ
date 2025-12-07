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

public class Vision extends SubsystemBase {

    // Cameras
    private final PhotonCamera camera1 = new PhotonCamera(CAMERA_NAME_1);
    private final PhotonCamera camera2 = new PhotonCamera(CAMERA_NAME_2);

    // Pose estimators
    private final PhotonPoseEstimator estimator1;
    private final PhotonPoseEstimator estimator2;

    // Telemetry state
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
        processCamera(camera1, estimator1);
        processCamera(camera2, estimator2);

        // Dashboard publishing (unchanged)
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

    private void processCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
        var results = camera.getAllUnreadResults();
        if (results.isEmpty()) return;

        var last = results.get(results.size() - 1);
        var maybeEst = estimator.update(last);
        if (maybeEst.isEmpty()) return;

        var visionEst = maybeEst.get();
        Pose2d visionPose = visionEst.estimatedPose.toPose2d();
        var targets = last.getTargets();

        if (targets.isEmpty()) return;

        int numTags = 0;
        double avgTagDist = 0.0;

        for (var tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;

            lastSeenTagId = tgt.getFiducialId();
            numTags++;
            avgTagDist += tagPose.get().toPose2d()
                    .getTranslation()
                    .getDistance(visionPose.getTranslation());
        }

        if (numTags == 0) return;

        avgTagDist /= numTags;

        Matrix<N3, N1> stdDevs = (numTags > 1)
                ? MULTI_TAG_STD_DEVS
                : SINGLE_TAG_STD_DEVS;

        // Ambiguity gating
        var bestTarget = last.getBestTarget();
        double ambiguity = bestTarget.getPoseAmbiguity();
        if (numTags == 1 && ambiguity > 0.25) return;

        // Distance-based scaling
        if (numTags == 1 && avgTagDist > 3.0) {
            stdDevs = VecBuilder.fill(
                    Double.MAX_VALUE,
                    Double.MAX_VALUE,
                    Double.MAX_VALUE);
        } else {
            double normDist = avgTagDist / 1.5;
            stdDevs = stdDevs.times(1.0 + normDist * normDist);
        }

        double ts = Utils.fpgaToCurrentTime(visionEst.timestampSeconds);

        // ✅ ✅ ✅ DIRECT FUSION (THIS IS THE CRITICAL PART)
        RobotContainer.m_drivetrain.addVisionMeasurement(
                visionPose,
                ts,
                stdDevs
        );

        // Store telemetry
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
