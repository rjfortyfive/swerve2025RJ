package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Effector;
import com.ctre.phoenix6.Utils;

public class ScoreL4L3L2 extends Command {
    private final Effector m_effector;

    private boolean coralGone = false;
    private double targetPositionLeft;
    private double targetPositionRight;
    private double positionMoveStartTime = 0.0;
    private static final double POSITION_TIMEOUT = 2.0; // Timeout in seconds for position movement
    private static final double SIM_POSITION_TIMEOUT = 0.5; // Shorter timeout for simulation
    
    public ScoreL4L3L2(Effector m_effector) {

        this.m_effector = m_effector;
        addRequirements(m_effector);

    }

    @Override
    public void initialize() {
        coralGone = false;
        positionMoveStartTime = 0.0;

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
            positionMoveStartTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public boolean isFinished() {
         // If we haven't reached "coral gone" yet, keep running
         if (!coralGone) return false;

         // Check if both motors are at or near their target
         if (m_effector.coralAtPosition(targetPositionLeft, targetPositionRight)) {
             return true;
         }
         
         // Timeout check - if we've been trying to reach position for too long, finish anyway
         if (positionMoveStartTime > 0) {
             double timeout = Utils.isSimulation() ? SIM_POSITION_TIMEOUT : POSITION_TIMEOUT;
             if ((Timer.getFPGATimestamp() - positionMoveStartTime) >= timeout) {
                 return true; // Timeout reached, finish command
             }
         }
         
         return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_effector.stop();
    }
    
}
