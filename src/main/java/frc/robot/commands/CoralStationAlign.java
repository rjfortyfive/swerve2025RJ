package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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

    // PID controller for rotation
    private final PIDController thetaController = new PIDController(2.0, 0, 0.1);
    
    // Tolerance for considering aligned (degrees)
    private static final double ANGLE_TOLERANCE_DEG = 3.0;
    
    // Maximum rotation rate (rad/s) - scaled down from max for smoother control
    private static final double MAX_ROT_RATE_RAD_PER_S = Units.degreesToRadians(180.0);

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
        thetaController.reset();
    }
    
    @Override
    public void execute() {
        Pose2d robotPose = m_drivetrain.getPose();
        double currentRad = robotPose.getRotation().getRadians();

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Calculate target angle based on alliance and station side
        // Blue: LEFT = +54°, RIGHT = -54°
        // Red:  LEFT = -54°, RIGHT = +54° (mirrored)
        double targetDeg;
        if (alliance == Alliance.Blue) {
            targetDeg = (m_stationSide == CoralStationSide.LEFT)
                        ? CORAL_STATION_ANGLE
                        : -CORAL_STATION_ANGLE;
        } else { // Red alliance
            targetDeg = (m_stationSide == CoralStationSide.LEFT)
                        ? -CORAL_STATION_ANGLE
                        : CORAL_STATION_ANGLE;
        }
        
        // Since we intake from the back, we need to point the REAR of the robot
        // toward the coral station. Add 180 degrees to the target angle.
        double targetDegRear = targetDeg + 180.0;
        // Normalize to -180 to 180 range
        targetDegRear = Math.IEEEremainder(targetDegRear, 360.0);
        double targetRad = Units.degreesToRadians(targetDegRear);

        // Calculate rotation rate using PID controller
        double rotRateRad = thetaController.calculate(currentRad, targetRad);
        
        // Clamp to maximum rotation rate
        rotRateRad = Math.max(-MAX_ROT_RATE_RAD_PER_S, Math.min(MAX_ROT_RATE_RAD_PER_S, rotRateRad));

        // Read current controller inputs for translation (allow driver to still translate)
        double velX, velY;
        if (RobotContainer.useXboxForDriver) {
            // Xbox controller: Left stick for translation
            velX = -RobotContainer.DriverXbox.getLeftY() * RobotContainer.MaxSpeed;
            velY = -RobotContainer.DriverXbox.getLeftX() * RobotContainer.MaxSpeed;
        } else {
            // Flight joystick
            velX = RobotContainer.DriverJoystick.getY() * RobotContainer.MaxSpeed;
            velY = RobotContainer.DriverJoystick.getX() * RobotContainer.MaxSpeed;
        }

        // Apply translation from controller + rotation from PID
        m_drivetrain.setControl(
            RobotContainer.drive
                .withVelocityX(velX)
                .withVelocityY(velY)
                .withRotationalRate(rotRateRad)
        );
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
