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

import com.ctre.phoenix6.Utils;

import frc.robot.RobotContainer;
import frc.robot.util.TagUtils;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

    // Cameras
    private final PhotonCamera camera1 = new PhotonCamera(CameraName1);
    private final PhotonCamera camera2 = new PhotonCamera(CameraName2);

    // Pose estimators
    private final PhotonPoseEstimator estimator1;
    private final PhotonPoseEstimator estimator2;

    // Std devs for estimator
    private Matrix<N3, N1> curStdDevs;

    // Track last seen tag ID
    private int lastSeenTagId = -1;

    private final CommandSwerveDrivetrain m_drivetrain;

    public Vision(CommandSwerveDrivetrain m_drivetrain) {
        this.m_drivetrain = m_drivetrain;

        camera1.setPipelineIndex(0);
        camera2.setPipelineIndex(0);

        estimator1 = new PhotonPoseEstimator(
                kTagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                RobotToCam1);

        estimator2 = new PhotonPoseEstimator(
                kTagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                RobotToCam2);

        estimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        estimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public int getLastSeenTagId() {
        return lastSeenTagId;
    }

    @Override
    public void periodic() {
        processFrame(camera1, 1, estimator1);
        processFrame(camera2, 2, estimator2);
    }

    private void processFrame(PhotonCamera cam, int camId, PhotonPoseEstimator estimator) {

        var results = cam.getAllUnreadResults();
        if (results.isEmpty()) return;

        var last = results.get(results.size() - 1);
        var maybeEst = estimator.update(last);
        if (maybeEst.isEmpty()) return;

        var visionEst = maybeEst.get();
        var visionPose = visionEst.estimatedPose.toPose2d();
        var targets = last.getTargets();

        // Determine std dev scaling
        Matrix<N3, N1> stdDevs = kSingleTagStdDevs;
        int numTags = 0;
        double avgTagDist = 0;

        for (var tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;

            lastSeenTagId = tgt.getFiducialId();
            numTags++;

            avgTagDist += tagPose.get().toPose2d().getTranslation()
                    .getDistance(visionPose.getTranslation());
        }

        if (numTags > 0) {
            avgTagDist /= numTags;
            if (numTags > 1) {
                stdDevs = kMultiTagStdDevs;
            }
            if (numTags == 1 && avgTagDist > 3.0) {
                stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                double normDist = avgTagDist / 1.5;
                double rampFactor = 1.0 + normDist * normDist;
                stdDevs = stdDevs.times(rampFactor);
            }
        } else {
            curStdDevs = kSingleTagStdDevs;
        }

        // Filter out estimates too far from odometry
        double distToRobot = RobotContainer.m_drivetrain.getPose().getTranslation()
                .getDistance(visionPose.getTranslation());

        if (distToRobot < 5.0) {
            RobotContainer.m_drivetrain.addVisionMeasurement(
                    visionPose,
                    Utils.fpgaToCurrentTime(visionEst.timestampSeconds),
                    stdDevs);
        }
    }

    /** Std devs of current vision estimate */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public int getClosestTagId() {
    Pose2d robotPose = m_drivetrain.getPose();  // you must store drivetrain reference
    List<Integer> allTags = Constants.Vision.kTags;
    
    return allTags.stream()
        .min(
            Comparator.comparingDouble(
                id -> TagUtils.getTagPose2d(id)
                    .map(tagPose -> tagPose.getTranslation()
                        .getDistance(robotPose.getTranslation()))
                    .orElse(Double.MAX_VALUE)
            )
        )
        .orElse(Constants.Vision.kTags.get(0));
}

    
}
