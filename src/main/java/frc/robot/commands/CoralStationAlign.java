package frc.robot.commands;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CoralStationAlign extends Command {
    public enum CoralStationSide { LEFT, RIGHT }
    private final CommandSwerveDrivetrain m_drivetrain;
    private final CoralStationSide m_stationSide;
    
    // Coral station angles (absolute field angles)
    // Blue: LEFT = +54°, RIGHT = -54°
    // Red:  LEFT = -54°, RIGHT = +54° (mirrored)
    private static final double CORAL_STATION_ANGLE = 54.0;

    // PID controller for rotation - increased gains for faster response
    private final PIDController thetaController = new PIDController(5.0, 0, 0.15);
    
    // Tolerance for considering aligned (degrees)
    private static final double ANGLE_TOLERANCE_DEG = 3.0;
    
    // Maximum rotation rate (rad/s) - use full robot capability for faster rotation
    // RobotContainer.MaxAngularRate is 1.25 rot/s = 450 deg/s, use a good portion of that
    private static final double MAX_ROT_RATE_RAD_PER_S = Units.degreesToRadians(360.0);
    
    // Use ApplyRobotSpeeds for direct control - this works even with zero translation
    // This is the same request type PathPlanner uses, so it handles rotation-only commands
    private final SwerveRequest.ApplyRobotSpeeds m_robotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();

    public CoralStationAlign(CommandSwerveDrivetrain drivetrain, CoralStationSide stationSide) {
        this.m_drivetrain = drivetrain;
        this.m_stationSide = stationSide;

        addRequirements(drivetrain);
        
        // Configure PID controller
        thetaController.setTolerance(Units.degreesToRadians(ANGLE_TOLERANCE_DEG));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void initialize() {
        // Reset PID controller and set initial setpoint
        thetaController.reset();
        
        // Get current pose and calculate target to initialize PID properly
        Pose2d robotPose = m_drivetrain.getPose();
        double currentRad = robotPose.getRotation().getRadians();
        double targetRad = calculateTargetAngle();
        
        // Set the setpoint explicitly to ensure PID is initialized
        thetaController.setSetpoint(targetRad);
        
        // Immediately start applying rotation to ensure it begins right away
        // Calculate initial rotation rate
        double rotRateRad = thetaController.calculate(currentRad, targetRad);
        double angleError = Math.abs(Math.IEEEremainder(currentRad - targetRad, 2 * Math.PI));
        
        // Apply minimum rotation rate if not at target
        if (angleError > Units.degreesToRadians(ANGLE_TOLERANCE_DEG)) {
            double minRotRate = Units.degreesToRadians(25.0); // Increased from 15 to 25 deg/s for faster initial rotation
            if (Math.abs(rotRateRad) < minRotRate) {
                double errorRad = Math.IEEEremainder(targetRad - currentRad, 2 * Math.PI);
                rotRateRad = Math.copySign(minRotRate, errorRad);
            }
        }
        rotRateRad = Math.max(-MAX_ROT_RATE_RAD_PER_S, Math.min(MAX_ROT_RATE_RAD_PER_S, rotRateRad));
        
        // Start rotation immediately using ApplyRobotSpeeds - this works with zero translation
        // Convert to field-centric speeds for the robot
        Pose2d currentPose = m_drivetrain.getPose();
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0.0, 0.0, rotRateRad, currentPose.getRotation()
        );
        m_drivetrain.setControl(m_robotSpeedsRequest.withSpeeds(chassisSpeeds));
    }
    
    /**
     * Calculates the target angle (in radians) for the rear of the robot to face the coral station
     */
    private double calculateTargetAngle() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Calculate target angle based on alliance and station side
        // LEFT and RIGHT are swapped: LEFT trigger should align to RIGHT station, RIGHT trigger to LEFT station
        // Blue: LEFT trigger (RIGHT station) = -54°, RIGHT trigger (LEFT station) = +54°
        // Red:  LEFT trigger (RIGHT station) = +54°, RIGHT trigger (LEFT station) = -54° (mirrored)
        double targetDeg;
        if (alliance == Alliance.Blue) {
            targetDeg = (m_stationSide == CoralStationSide.LEFT)
                        ? -CORAL_STATION_ANGLE  // LEFT trigger -> RIGHT station
                        : CORAL_STATION_ANGLE;  // RIGHT trigger -> LEFT station
        } else { // Red alliance
            targetDeg = (m_stationSide == CoralStationSide.LEFT)
                        ? CORAL_STATION_ANGLE   // LEFT trigger -> RIGHT station
                        : -CORAL_STATION_ANGLE; // RIGHT trigger -> LEFT station
        }

        // Since we intake from the back, we need to point the REAR of the robot
        // toward the coral station. The station angles are the direction TO the station
        // from the origin, so we need to reverse it (add 180) to point the rear toward it.
        // However, if heading is reversed, try without the 180 offset first.
        // Remove 180 degree offset - use station angle directly
        // (If this doesn't work, we may need to swap LEFT/RIGHT logic instead)
        double targetDegRear = targetDeg;
        // Normalize to -180 to 180 range
        targetDegRear = Math.IEEEremainder(targetDegRear, 360.0);
        return Units.degreesToRadians(targetDegRear);
    }
    
    @Override
    public void execute() {
        Pose2d robotPose = m_drivetrain.getPose();
        double currentRad = robotPose.getRotation().getRadians();
        double targetRad = calculateTargetAngle();

        // Update setpoint in case alliance changed
        thetaController.setSetpoint(targetRad);
        
        // Calculate rotation rate using PID controller
        double rotRateRad = thetaController.calculate(currentRad, targetRad);
        
        // Calculate angle error for minimum rotation rate logic
        double angleError = Math.abs(Math.IEEEremainder(currentRad - targetRad, 2 * Math.PI));
        
        // Apply minimum rotation rate if we're not at target (prevents deadband issues)
        // This ensures rotation happens even when PID output is zero or very small
        if (angleError > Units.degreesToRadians(ANGLE_TOLERANCE_DEG)) {
            // Not at target - ensure we always have a minimum rotation rate
            double minRotRate = Units.degreesToRadians(25.0); // Increased from 15 to 25 deg/s for faster rotation
            double desiredRotRate = rotRateRad;
            
            // If PID output is zero or too small, calculate direction and apply minimum
            if (Math.abs(desiredRotRate) < minRotRate) {
                // Determine rotation direction based on shortest path to target
                double errorRad = Math.IEEEremainder(targetRad - currentRad, 2 * Math.PI);
                desiredRotRate = Math.copySign(minRotRate, errorRad);
            }
            rotRateRad = desiredRotRate;
        }
        
        // Clamp to maximum rotation rate
        rotRateRad = Math.max(-MAX_ROT_RATE_RAD_PER_S, Math.min(MAX_ROT_RATE_RAD_PER_S, rotRateRad));

        // Read current controller inputs for translation (allow driver to still translate)
        // Apply deadbands to match default command behavior (1% of MaxSpeed)
        double deadband = RobotContainer.MaxSpeed * 0.01;
        
        double rawVelX, rawVelY;
        if (RobotContainer.useXboxForDriver) {
            // Xbox controller: Left stick for translation
            rawVelX = -RobotContainer.DriverXbox.getLeftY() * RobotContainer.MaxSpeed;
            rawVelY = -RobotContainer.DriverXbox.getLeftX() * RobotContainer.MaxSpeed;
        } else {
            // Flight joystick
            rawVelX = RobotContainer.DriverJoystick.getY() * RobotContainer.MaxSpeed;
            rawVelY = RobotContainer.DriverJoystick.getX() * RobotContainer.MaxSpeed;
        }
        
        // Apply deadband to translation speeds to match default command
        double velX = Math.abs(rawVelX) > deadband ? rawVelX : 0.0;
        double velY = Math.abs(rawVelY) > deadband ? rawVelY : 0.0;

        // Apply translation from controller + rotation from PID
        // Use ApplyRobotSpeeds for direct chassis speed control
        // This works even when translation is zero, allowing rotation-only commands
        Pose2d currentPose = m_drivetrain.getPose();
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            velX, velY, rotRateRad, currentPose.getRotation()
        );
        m_drivetrain.setControl(m_robotSpeedsRequest.withSpeeds(chassisSpeeds));
    }
    
    @Override
    public boolean isFinished() {
        // Never finish automatically - runs while button is held (whileTrue)
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        // Don't set control here - let the default command take over
    }
}
