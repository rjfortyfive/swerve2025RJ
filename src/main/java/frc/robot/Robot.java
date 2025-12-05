package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.pathplanner.lib.commands.PathfindingCommand;

import au.grapplerobotics.CanBridge;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    public Robot() {
        // Initialize robot components
        var inst = NetworkTableInstance.getDefault();
        inst.startServer();
        m_robotContainer = new RobotContainer();
        PathfindingCommand.warmupCommand().schedule();

        CanBridge.runTCP();
    }

    

    @Override
    public void robotPeriodic() {
        // Publish Match Timer to Elastic
        double matchTime = DriverStation.getMatchTime();

        NetworkTableInstance.getDefault()
            .getTable("FMSInfo")
            .getEntry("MatchTime")
            .setDouble(matchTime);       
       
        // Runs the scheduler for commands
        CommandScheduler.getInstance().run();
        
    }
    @Override
    public void robotInit() {
        //Load Elastic Dashboard
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    

    @Override
    public void disabledPeriodic() {
        // Update vision while disabled
        m_robotContainer.m_vision.periodic();
    }

    @Override
    public void autonomousInit() {
        // Start the selected autonomous command
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        Constants.setL4();

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void disabledInit() {

    }

}