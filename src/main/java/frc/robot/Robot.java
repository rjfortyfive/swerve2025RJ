package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Lights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.commands.PathfindingCommand;

import au.grapplerobotics.CanBridge;

import com.ctre.phoenix6.controls.NeutralOut;

import frc.robot.subsystems.Hang;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private Vision vision;
    private CommandSwerveDrivetrain drivetrain;
    private final Lights lights = new Lights();

    private final NeutralOut m_brake = new NeutralOut();

    Timer gc = new Timer();


    public Robot() {
        gc.start();
        // Initialize robot components
        var inst = NetworkTableInstance.getDefault();
        inst.startServer();
        m_robotContainer = new RobotContainer();
        PathfindingCommand.warmupCommand().schedule();

        CanBridge.runTCP();
    }

    

    @Override
    public void robotPeriodic() {
        // Runs the scheduler for commands
        CommandScheduler.getInstance().run();
        vision.periodic();
        drivetrain.periodic();
        
    }
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        drivetrain = m_robotContainer.drivetrain;
        vision = m_robotContainer.vision;
    }

    

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        // Start the selected autonomous command
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        // RobotContainer.drivetrain.seedFieldCentric(); // Not currently working -
        // reverses direction on blue side
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // resetPose();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        Constants.setL4();
        lights.lightsOn(Lights.purpleGoldStep);
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void disabledInit() {
        Hang.brakeHang();
    }

    public void resetPose() {
        
        
        Pose2d startPose = new Pose2d(14, 6, Rotation2d.fromDegrees(60)); // Add your third pose here


        drivetrain.resetPose(startPose);
        System.out.println("Resetting pose to " + startPose);
    }

}