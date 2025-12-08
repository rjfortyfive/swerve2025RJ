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

public class MakeGoToTag extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final Vision m_vision;
    private final tagSide m_side;
    private final double m_offsetMeters;
    private final double m_frontOffsetMeters;

    private Command currentPathCommand = null;

    // How often to refresh the path (in seconds)
    private static final double UPDATE_PERIOD = 0.25; 
    private double timer = 0.0;

    private final PathConstraints constraints =
        new PathConstraints(
            Pathfinding.MAX_SPEED,
            Pathfinding.MAX_ACCEL,
            Math.toRadians(Pathfinding.MAX_ROT_SPEED),
            Math.toRadians(Pathfinding.MAX_ROT_ACCEL));

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

        timer = 0.0;
        updatePath();   // Build initial path immediately
    }


    @Override
    public void execute() {
        timer += 0.02;
        if (timer >= UPDATE_PERIOD) {
            timer = 0.0;
            updatePath();
        }
    }


    private void updatePath() {
        Pose2d robotPose = m_drivetrain.getPose();
    
        int tagId = m_vision.getClosestTagId(robotPose);
        if (tagId < 0) return;
    
        Pose2d goal = TagUtils.computeTagAdjacencyPose(
            tagId, m_side, m_offsetMeters, m_frontOffsetMeters);
    
        System.out.println("GoToTag goal = " + goal);
    
        // Cancel existing path command if it's still running
        if (currentPathCommand != null && currentPathCommand.isScheduled()) {
            currentPathCommand.cancel();
        }
    
        currentPathCommand = AutoBuilder.pathfindToPose(
            goal,
            constraints,
            0.0
        );
    
        currentPathCommand.schedule();
    }
    


    @Override
    public void end(boolean interrupted) {
        if (currentPathCommand != null) {
            currentPathCommand.cancel();
            currentPathCommand = null;
        }
    }


    @Override
    public boolean isFinished() {
        return false; // ends when button is released
    }
}
