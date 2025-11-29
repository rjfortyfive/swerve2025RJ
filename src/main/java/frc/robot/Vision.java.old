package frc.robot;

import static frc.robot.Constants.Vision.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import com.ctre.phoenix6.Utils;

public class Vision {
    private final PhotonCamera camera1 = new PhotonCamera(CameraName1);
    private final PhotonCamera camera2 = new PhotonCamera(CameraName2);

    // private final PhotonCamera camera1 = new PhotonCamera(kCameraName + "5");

    private final PhotonPoseEstimator estimator1;
    private final PhotonPoseEstimator estimator2;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;

    // ID of the last AprilTag seen (or -1 if none)
    private int lastSeenTagId = -1;

    /** @return the fiducial ID of the last tag processed, or -1 if none */
    public int getLastSeenTagId() {
        Logger.warn("Last Tag Id [{}]", lastSeenTagId);
        return lastSeenTagId;
    }

    /**
     * @param estConsumer Lamba that will accept a pose estimate and pass it to your
     *                    desired {@link
     *                    edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */

    public Vision(EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;

        // Select the PhotonVision pipeline (0-based index) to use for processing

        // camera1.setPipelineIndex(0);

        camera1.setPipelineIndex(0);
        camera2.setPipelineIndex(0);

        estimator1 = new PhotonPoseEstimator(
                Constants.Vision.kTagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                RobotToCam1);
        estimator2 = new PhotonPoseEstimator(
                Constants.Vision.kTagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                RobotToCam2);

        estimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        estimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Initialize NetworkTables entries for external monitoring
    }

    public void periodic() {
        // Always fetch the latest pipeline result to ensure we see current detections
        processFrame(camera1, 1, estimator1);
        processFrame(camera2, 2, estimator2);

    }

    private void processFrame(PhotonCamera cam, int camId, PhotonPoseEstimator estimator) {
        var results = cam.getAllUnreadResults();
        Logger.debug("Cam {} pipeline results [{}]", camId, results.size());
        if (results.isEmpty()) {
            Logger.debug("Cam {} pipeline no results", camId);
            return;
        }

        var last = results.get(results.size() - 1);
        var maybeEst = estimator.update(last);
        if (maybeEst.isEmpty()) {
            Logger.debug("Cam {} no targets detected this cycle", camId);
            return;
        }

        var visionEst = maybeEst.get();
        var visionPose = visionEst.estimatedPose.toPose2d();
        var targets = last.getTargets();

        // Heuristic: pick σ based on # tags & average tag‐to‐vision distance
        Matrix<N3, N1> stdDevs = kSingleTagStdDevs;
        int numTags = 0;
        double avgTagDist = 0;
        for (var tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(
                    tgt.getFiducialId());
            if (tagPose.isEmpty()) {
                Logger.warn("tagPose is empty");
                continue;
            }
            // record which tag we just processed
            lastSeenTagId = tgt.getFiducialId();
            numTags++;
            avgTagDist += tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(visionPose.getTranslation());
            ;
        }

        if (numTags > 0) {
            avgTagDist /= numTags;
            if (numTags > 1) {
                stdDevs = kMultiTagStdDevs;
            }
            if (numTags == 1 && avgTagDist > 3.0) {
                stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                // Steeper quadratic ramp: normalized distance over 1.5 m, squared
                double normDist = avgTagDist / 1.5;
                double rampFactor = 1.0 + normDist * normDist;
                stdDevs = stdDevs.times(rampFactor);
            }
        } else {
            // No tags visible. Default to single-tag std devs
            Logger.debug("no tags visible");
            curStdDevs = kSingleTagStdDevs;
        }

        // Now check how far visionPose is from our current odometry pose:
        double distanceToRobot = frc.robot.RobotContainer.drivetrain.getPose().getTranslation()
                .getDistance(visionEst.estimatedPose.toPose2d().getTranslation());
        if (distanceToRobot < 5) {
            Logger.debug("Cam {} fusing vision ({} tags, avgTagDist={:.2f}, robotDist={:.2f})",
                    camId, numTags, avgTagDist, distanceToRobot);

            RobotContainer.drivetrain.addVisionMeasurement(
                    visionPose,
                    Utils.fpgaToCurrentTime(visionEst.timestampSeconds),
                    stdDevs);
        } else {
            Logger.debug("Cam {} skipping fusion; {:.2f} m away", camId, distanceToRobot);
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        Logger.debug("current std dev [{}]", curStdDevs);
        return curStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}