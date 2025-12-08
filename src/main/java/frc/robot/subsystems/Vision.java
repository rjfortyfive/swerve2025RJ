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

// PhotonVision simulation imports
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.SimCameraProperties;
import edu.wpi.first.math.geometry.Transform3d;

public class Vision extends SubsystemBase {

    // Cameras
    private final PhotonCamera camera1 = new PhotonCamera(CAMERA_NAME_1);
    private final PhotonCamera camera2 = new PhotonCamera(CAMERA_NAME_2);

    // Pose estimators
    private final PhotonPoseEstimator estimator1;
    private final PhotonPoseEstimator estimator2;

    // Simulation support
    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim1;
    private PhotonCameraSim cameraSim2;

    // Telemetry state
    private Pose2d latestPose;
    private Matrix<N3, N1> latestStdDevs;
    private double latestTimestamp = -1.0;
    private String latestCameraName = "";
    private int lastSeenTagId = -1;
    private double latestAmbiguity = 1.0;
    private int latestNumTags = 0;
    
    // Rate limiting for vision fusion
    private double lastVisionFusionTime = 0.0;
    private static final double MIN_VISION_UPDATE_INTERVAL = 0.1; // Minimum 100ms between vision fusions

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
        
        // Setup simulation if in sim mode
        if (Utils.isSimulation()) {
            setupSimulation();
        }
    }
    
    /**
     * Sets up PhotonVision simulation
     */
    private void setupSimulation() {
        // Create vision system sim
        visionSim = new VisionSystemSim("main");
        
        // Add AprilTag field layout
        visionSim.addAprilTags(TAG_LAYOUT);
        
        // Create camera properties
        SimCameraProperties cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(960, 720, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90));
        cameraProps.setFPS(30);
        cameraProps.setAvgLatencyMs(50);
        cameraProps.setLatencyStdDevMs(15);
        
        // Create simulated cameras
        cameraSim1 = new PhotonCameraSim(camera1, cameraProps);
        cameraSim2 = new PhotonCameraSim(camera2, cameraProps);
        
        // Add cameras to vision sim with their robot-to-camera transforms
        visionSim.addCamera(cameraSim1, ROBOT_TO_CAM_1);
        visionSim.addCamera(cameraSim2, ROBOT_TO_CAM_2);
    }

    @Override
    public void periodic() {
        // Update vision simulation if in sim mode
        if (Utils.isSimulation() && visionSim != null) {
            // Update vision sim with current robot pose
            Pose2d robotPose = RobotContainer.m_drivetrain.getPose();
            visionSim.update(robotPose);
        }
        
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

        // Get current robot pose estimate and speeds
        Pose2d currentPose = RobotContainer.m_drivetrain.getPose();
        var driveState = RobotContainer.m_drivetrain.getState();
        var speeds = driveState.Speeds;
        
        // Calculate robot velocity magnitude
        double velocityMagnitude = Math.sqrt(
            speeds.vxMetersPerSecond * speeds.vxMetersPerSecond +
            speeds.vyMetersPerSecond * speeds.vyMetersPerSecond
        );
        double angularVelocityMagnitude = Math.abs(speeds.omegaRadiansPerSecond);
        
        // If robot is stationary (or nearly stationary), don't fuse vision to prevent drift
        // This is especially important in simulation where vision noise can cause drift
        double velocityThreshold = Utils.isSimulation() ? 0.05 : 0.02; // 5cm/s in sim, 2cm/s on robot
        double angularThreshold = Utils.isSimulation() ? Math.toRadians(2.0) : Math.toRadians(1.0);
        
        if (velocityMagnitude < velocityThreshold && angularVelocityMagnitude < angularThreshold) {
            // Robot is stationary, skip vision fusion to prevent drift
            return;
        }
        
        // Calculate difference between vision pose and current pose
        double poseDifference = visionPose.getTranslation().getDistance(currentPose.getTranslation());
        double rotationDifference = Math.abs(visionPose.getRotation().minus(currentPose.getRotation()).getRadians());
        
        // Rate limiting - don't fuse vision too frequently
        double currentTime = Utils.fpgaToCurrentTime(visionEst.timestampSeconds);
        if (currentTime - lastVisionFusionTime < MIN_VISION_UPDATE_INTERVAL) {
            return; // Skip if too soon since last fusion
        }
        
        // Only fuse vision measurements if there's a meaningful difference
        // This prevents drift when robot is stationary
        // Stricter thresholds to prevent drift
        double translationThreshold = Utils.isSimulation() ? 0.20 : 0.10; // Even stricter thresholds
        double rotationThreshold = Utils.isSimulation() ? Math.toRadians(10.0) : Math.toRadians(5.0);
        
        if (poseDifference < translationThreshold && rotationDifference < rotationThreshold) {
            // Vision pose is too close to current estimate, skip fusion to prevent drift
            return;
        }
        
        // In simulation, increase standard deviations significantly to reduce vision weight
        // This makes vision much less influential and reduces drift
        if (Utils.isSimulation()) {
            stdDevs = stdDevs.times(3.0); // Triple the uncertainty in simulation
        }

        double ts = Utils.fpgaToCurrentTime(visionEst.timestampSeconds);
        
        // Update last fusion time
        lastVisionFusionTime = currentTime;

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
