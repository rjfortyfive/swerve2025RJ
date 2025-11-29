package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.TagUtils;
import frc.robot.util.TagUtils.tagSide;
import frc.robot.Constants;
import frc.robot.Logger;

/**
 * Drives the robot to a specific tag adjacency pose using PathPlanner's pathfinding.
 * No cancel-on-stick logic included — command ends when the path finishes normally.
 */
public class MakeGoToTag extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final int m_tagId;
    private final tagSide m_side;
    private final double m_offsetMeters;
    private final double m_frontOffsetMeters;

    private Command m_pathCommand; // this is the PathPlanner-generated command

    public MakeGoToTag(
            CommandSwerveDrivetrain drivetrain,
            int tagId,
            tagSide side,
            double offsetMeters,
            double frontOffsetMeters) {

        m_drivetrain = drivetrain;
        m_tagId = tagId;
        m_side = side;
        m_offsetMeters = offsetMeters;
        m_frontOffsetMeters = frontOffsetMeters;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        // 1. Compute target pose
        Pose2d goal = TagUtils.computeTagAdjacencyPose(
            m_tagId,
            m_side,
            m_offsetMeters,
            m_frontOffsetMeters
        );

        Logger.debug("GoToTagCommand — Computed target pose: {}", goal);

        // 2. Build PathPlanner pathfinding command
        PathConstraints constraints = Constants.Pathfinding.kPathConstraints;

        m_pathCommand = AutoBuilder.pathfindToPose(
            goal,
            constraints,
            0.0 // goal end velocity
        );

        // 3. Start executing the generated path command
        m_pathCommand.initialize();
    }

    @Override
    public void execute() {
        if (m_pathCommand != null) {
            m_pathCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (m_pathCommand != null) {
            m_pathCommand.end(interrupted);
        }

        Logger.debug("GoToTagCommand ended. Interrupted = {}", interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_pathCommand != null && m_pathCommand.isFinished();
    }
}
