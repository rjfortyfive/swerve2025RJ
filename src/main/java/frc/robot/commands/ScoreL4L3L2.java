package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Effector;

public class ScoreL4L3L2 extends Command {
    private final Effector m_effector;

    private boolean coralGone = false;
    private double targetPositionLeft;
    private double targetPositionRight;
    
    public ScoreL4L3L2(Effector m_effector) {

        this.m_effector = m_effector;
        addRequirements(m_effector);

    }

    @Override
    public void initialize() {
        coralGone = false;

        // Begin normal outtake
        m_effector.start(40);
    }
    @Override
    public void execute() {
        if (!m_effector.isCoralDetected() && !coralGone) {
            coralGone = true;

            // Determine new target positions from current positions
            double leftCurrent  = m_effector.getLeftPosition();
            double rightCurrent = m_effector.getRightPosition();

            // Outtake = reverse lock direction (your right motor is inverted)
            targetPositionLeft  = leftCurrent  + Constants.effector.SCORE_ROTATIONS;
            targetPositionRight = rightCurrent - Constants.effector.SCORE_ROTATIONS;

            // Command motion magic to rotate additional amount
            m_effector.moveToPositions(targetPositionLeft, targetPositionRight);
        }
    }

    @Override
    public boolean isFinished() {
         // If we haven't reached "coral gone" yet, keep running
         if (!coralGone) return false;

         // Check if both motors are at or near their target
         return m_effector.coralAtPosition(targetPositionLeft, targetPositionRight);
    }

    @Override
    public void end(boolean interrupted) {
        m_effector.stop();
    }
    
}
