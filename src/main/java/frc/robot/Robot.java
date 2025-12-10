package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.pathplanner.lib.commands.PathfindingCommand;

import au.grapplerobotics.CanBridge;

/**
 * Main robot class that extends TimedRobot.
 * 
 * This class handles robot initialization and periodic updates, including:
 * - Initializing subsystems and controllers
 * - Running the command scheduler
 * - Managing autonomous and teleop modes
 * - Publishing match timer to Elastic Dashboard
 * 
 * The robot uses a command-based architecture where subsystems are controlled
 * through commands scheduled via the CommandScheduler.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    /**
     * Constructor called when robot code starts.
     * Initializes network tables, creates RobotContainer (which sets up all subsystems
     * and bindings), and warms up PathPlanner pathfinding.
     */
    public Robot() {
        // Start NetworkTables server for robot-to-driver station communication
        var inst = NetworkTableInstance.getDefault();
        inst.startServer();
        
        // Create RobotContainer - this initializes all subsystems, commands, and bindings
        m_robotContainer = new RobotContainer();
        
        // Warm up PathPlanner pathfinding to reduce first-use delay
        PathfindingCommand.warmupCommand().schedule();

        // Start CAN bridge for CANivore communication
        CanBridge.runTCP();
    }

    

    /**
     * Called every robot code loop, regardless of mode.
     * Runs the command scheduler and publishes match timer to NetworkTables.
     */
    @Override
    public void robotPeriodic() {
        // Publish match timer to Elastic Dashboard via NetworkTables
        double matchTime = DriverStation.getMatchTime();
        NetworkTableInstance.getDefault()
            .getTable("FMSInfo")
            .getEntry("MatchTime")
            .setDouble(matchTime);       
        
        // Run the command scheduler - this executes all scheduled commands
        CommandScheduler.getInstance().run();
    }
    
    /**
     * Called once when the robot first starts up.
     * Starts the Elastic Dashboard web server and sets elevator L4 level based on alliance.
     */
    @Override
    public void robotInit() {
        // Allow Export from Robot for Elastic Dashboard
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        
        // Set L4 elevator height and vision tags based on alliance color
        Constants.setL4();
    }

    

    /**
     * Called periodically while robot is disabled.
     * Continues updating vision to maintain pose estimate even when disabled.
     */
    @Override
    public void disabledPeriodic() {
        // Continue vision processing to update robot pose estimate
        // This helps maintain accurate position when robot is re-enabled
        m_robotContainer.m_vision.periodic();
    }

    /**
     * Called once when autonomous mode is entered.
     * Gets the selected autonomous command from the chooser and schedules it.
     */
    @Override
    public void autonomousInit() {
        // Get the autonomous command selected from the SendableChooser
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * Called periodically during autonomous mode.
     * Command scheduler handles all robot control, so this is typically empty.
     */
    @Override
    public void autonomousPeriodic() {
        // Command scheduler handles all periodic tasks
    }

    /**
     * Called once when teleop mode is entered.
     * Cancels any running autonomous command to ensure clean transition.
     */
    @Override
    public void teleopInit() {
        // Cancel autonomous command if still running
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * Called periodically during teleop mode.
     * Command scheduler handles all robot control, so this is typically empty.
     */
    @Override
    public void teleopPeriodic() {
        // Command scheduler handles all periodic tasks
    }

    /**
     * Called once when robot enters disabled mode.
     */
    @Override
    public void disabledInit() {
        // Initialization tasks when robot becomes disabled
    }

}