package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Effector;

/**
 * Scoring command for levels L2, L3, and L4.
 * 
 * This command implements a 2-phase scoring sequence:
 * 
 * Phase 1 - Outtake: Effector runs forward at 40 RPS until coral is ejected
 *   (sensor no longer detects coral)
 * 
 * Phase 2 - Lock Rotation: Once coral is gone, rotate effector wheels by
 *   SCORE_ROTATIONS (4 rotations) in opposite directions to ensure complete ejection.
 *   This is done using MotionMagic position control.
 * 
 * Command completes when both effector motors reach their target positions.
 */
public class ScoreL4L3L2 extends Command {
    private final Effector m_effector;

    /** Tracks whether coral has been ejected (phase 1 complete) */
    private boolean coralGone = false;
    
    /** Target position for left effector motor after lock rotation */
    private double targetPositionLeft;
    /** Target position for right effector motor after lock rotation */
    private double targetPositionRight;
    
    /**
     * Creates a new ScoreL4L3L2 command.
     * 
     * @param m_effector Effector subsystem to control
     */
    public ScoreL4L3L2(Effector m_effector) {
        this.m_effector = m_effector;
        addRequirements(m_effector);
    }

    @Override
    public void initialize() {
        // Reset state
        coralGone = false;

        // Phase 1: Start outtake at 40 RPS to eject coral
        m_effector.start(40);
    }
    
    @Override
    public void execute() {
        if (!m_effector.isCoralDetected() && !coralGone) {
            // Coral has been ejected - transition to Phase 2
            coralGone = true;

            // Get current motor positions
            double leftCurrent  = m_effector.getLeftPosition();
            double rightCurrent = m_effector.getRightPosition();

            // Calculate target positions by rotating additional SCORE_ROTATIONS
            // Left motor: rotate forward (+SCORE_ROTATIONS)
            // Right motor: rotate backward (-SCORE_ROTATIONS) because it's inverted
            targetPositionLeft  = leftCurrent  + Constants.effector.SCORE_ROTATIONS;
            targetPositionRight = rightCurrent - Constants.effector.SCORE_ROTATIONS;

            // Switch to position control to complete the lock rotation
            m_effector.moveToPositions(targetPositionLeft, targetPositionRight);
        }
        // If coral is still detected, continue outtake phase
    }

    @Override
    public boolean isFinished() {
         // Keep running until coral is ejected
         if (!coralGone) return false;

         // Once coral is gone, wait for motors to reach lock rotation positions
         return m_effector.coralAtPosition(targetPositionLeft, targetPositionRight);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop effector motors
        m_effector.stop();
    }
    
}
