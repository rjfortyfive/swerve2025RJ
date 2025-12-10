package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.util.TagUtils;
import frc.robot.util.TagUtils.tagSide;

import frc.robot.Constants.Pathfinding;

/**
 * Command that continuously paths to the closest AprilTag on the specified side.
 * 
 * This command is used for strafing/aligning to scoring positions. It:
 * 1. Finds the closest visible AprilTag using vision
 * 2. Calculates a goal pose offset from that tag (on the specified side)
 * 3. Uses PathPlanner to generate and follow a path to that pose
 * 4. Refreshes the path every 0.25 seconds to account for robot movement
 * 
 * The command runs continuously while the button is held and automatically
 * updates the target as the robot moves or the closest tag changes.
 */
public class MakeGoToTag extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final Vision m_vision;
    /** Which side of the tag to align to (LEFT or RIGHT) */
    private final tagSide m_side;
    /** Lateral offset from tag in meters (left/right) */
    private final double m_offsetMeters;
    /** Front offset from tag in meters (toward/away from reef) */
    private final double m_frontOffsetMeters;

    /** Currently running pathfinding command (refreshed periodically) */
    private Command currentPathCommand = null;

    /** How often to refresh the path in seconds */
    private static final double UPDATE_PERIOD = 0.25;
    /** Timer tracking time since last path update */
    private double timer = 0.0;

    private final PathConstraints constraints =
        new PathConstraints(
            Pathfinding.MAX_SPEED,
            Pathfinding.MAX_ACCEL,
            Math.toRadians(Pathfinding.MAX_ROT_SPEED),
            Math.toRadians(Pathfinding.MAX_ROT_ACCEL));

    /**
     * Creates a new MakeGoToTag command.
     * 
     * @param drivetrain Drivetrain subsystem for path following
     * @param vision Vision subsystem for finding closest AprilTag
     * @param side Which side of the tag to align to (LEFT or RIGHT)
     * @param offsetMeters Lateral offset from tag in meters
     * @param frontOffsetMeters Front offset from tag in meters
     */
    public MakeGoToTag(
            CommandSwerveDrivetrain drivetrain,
            Vision vision,
            tagSide side,
            double offsetMeters,
            double frontOffsetMeters) {

        this.m_drivetrain = drivetrain;
        this.m_vision = vision;
        this.m_side = side;
        this.m_offsetMeters = offsetMeters;
        this.m_frontOffsetMeters = frontOffsetMeters;
    }

    @Override
    public void initialize() {
        System.out.println("MakeGoToTag INITIALIZED");

        // Reset timer and generate initial path immediately
        timer = 0.0;
        updatePath();
    }

    @Override
    public void execute() {
        // Increment timer by 20ms (typical robot loop period)
        timer += 0.02;
        
        // Refresh path every UPDATE_PERIOD seconds
        if (timer >= UPDATE_PERIOD) {
            timer = 0.0;
            updatePath();
        }
    }

    /**
     * Updates the pathfinding goal based on the closest visible AprilTag.
     * Cancels the current path command and starts a new one to the updated goal.
     */
    private void updatePath() {
        // Get current robot pose
        Pose2d robotPose = m_drivetrain.getPose();
    
        // Find closest visible AprilTag
        int tagId = m_vision.getClosestTagId(robotPose);
        if (tagId < 0) return;  // No tags visible, skip update
    
        // Calculate goal pose offset from the tag
        Pose2d goal = TagUtils.computeTagAdjacencyPose(
            tagId, m_side, m_offsetMeters, m_frontOffsetMeters);
    
        System.out.println("GoToTag goal = " + goal);
    
        // Cancel existing path command if it's still running
        if (currentPathCommand != null && currentPathCommand.isScheduled()) {
            currentPathCommand.cancel();
        }
    
        // Generate new path to the goal and schedule it
        currentPathCommand = AutoBuilder.pathfindToPose(
            goal,
            constraints,
            0.0  // Goal end velocity (0 = stop at goal)
        );
    
        currentPathCommand.schedule();
    }
    


    @Override
    public void end(boolean interrupted) {
        // Cancel any running path command when this command ends
        if (currentPathCommand != null) {
            currentPathCommand.cancel();
            currentPathCommand = null;
        }
    }

    @Override
    public boolean isFinished() {
        // Command runs continuously until button is released (command is cancelled)
        return false;
    }
}
