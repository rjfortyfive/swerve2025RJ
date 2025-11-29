package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.util.TagUtils;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

    private final PhotonCamera camera1 = new PhotonCamera(CameraName1);
    private final PhotonCamera camera2 = new PhotonCamera(CameraName2);

    private final PhotonPoseEstimator estimator1;
    private final PhotonPoseEstimator estimator2;

    private Pose2d latestPose;
    private Matrix<N3, N1> latestStdDevs;
    private int lastSeenTagId = -1;

    public Vision() {
        camera1.setPipelineIndex(0);
        camera2.setPipelineIndex(0);

        estimator1 = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, RobotToCam1);
        estimator2 = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, RobotToCam2);

        estimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        estimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        updateCamera(estimator1, camera1);
        updateCamera(estimator2, camera2);
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

        // Std dev calculation
        Matrix<N3, N1> stdDevs = kSingleTagStdDevs;
        int numTags = 0;
        double avgTagDist = 0.0;

        for (var tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;

            lastSeenTagId = tgt.getFiducialId();
            numTags++;
            avgTagDist += tagPose.get().toPose2d().getTranslation().getDistance(visionPose.getTranslation());
        }

        if (numTags > 0) {
            avgTagDist /= numTags;
            if (numTags > 1) stdDevs = kMultiTagStdDevs;
            if (numTags == 1 && avgTagDist > 3.0) {
                stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                double normDist = avgTagDist / 1.5;
                stdDevs = stdDevs.times(1.0 + normDist * normDist);
            }
        }

        latestPose = visionPose;
        latestStdDevs = stdDevs;
    }

    public Pose2d getLatestPose() {
        return latestPose;
    }

    public Matrix<N3, N1> getLatestStdDevs() {
        return latestStdDevs;
    }

    public int getLastSeenTagId() {
        return lastSeenTagId;
    }

    public int getClosestTagId(Pose2d robotPose) {
        List<Integer> allTags = Constants.Vision.kTags;
        return allTags.stream()
                .min(Comparator.comparingDouble(id ->
                        TagUtils.getTagPose2d(id)
                                .map(tagPose -> tagPose.getTranslation().getDistance(robotPose.getTranslation()))
                                .orElse(Double.MAX_VALUE)))
                .orElse(Constants.Vision.kTags.get(0));
    }
}

