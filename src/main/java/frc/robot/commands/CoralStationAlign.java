package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CoralStationAlign extends Command {
    public enum CoralStationSide { LEFT, RIGHT }
    private final CommandSwerveDrivetrain m_drivetrain;
    private final CoralStationSide m_stationSide;
    
        // Core angle (mirrored per alliance)
    private static final double CORAL_STATION_ANGLE = 54.0;

    private static final double ANGLE_TOLERANCE_DEG = 5.0;

    public CoralStationAlign(CommandSwerveDrivetrain drivetrain, CoralStationSide stationSide) {
        this.m_drivetrain = drivetrain;
        this.m_stationSide = stationSide;

        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        // Nothing to initialize
    }
    
    @Override
    public void execute() {
        
        Pose2d pose = m_drivetrain.getPose();
        double currentDeg = pose.getRotation().getDegrees();

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Angle pulled from Pathplanner
        // Blue: LEFT = +54, RIGHT = –54
        // Red:  Mirror the field → LEFT = –54, RIGHT = +54
        double targetDeg;
        if (alliance == Alliance.Blue) {
            targetDeg = (m_stationSide == CoralStationSide.LEFT)
                        ?  CORAL_STATION_ANGLE
                        : -CORAL_STATION_ANGLE;
        } else { // Red alliance
            targetDeg = (m_stationSide == CoralStationSide.LEFT)
                        ? -CORAL_STATION_ANGLE
                        :  CORAL_STATION_ANGLE;
        }

        double errorDeg = Math.IEEEremainder(targetDeg - currentDeg, 360);
        double rotRateRad = (Math.abs(errorDeg) < ANGLE_TOLERANCE_DEG)
                ? 0.0
                : Math.copySign(Constants.Pathfinding.MAX_ROT_SPEED, errorDeg);

        m_drivetrain.setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotRateRad)
        );
    };
    
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        m_drivetrain.setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    };
}
