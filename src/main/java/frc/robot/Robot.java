package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Lights;

import com.pathplanner.lib.commands.PathfindingCommand;

import au.grapplerobotics.CanBridge;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private final Lights lights = new Lights();


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


        
    }
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
     //   vision = m_robotContainer.vision;
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

    }

}