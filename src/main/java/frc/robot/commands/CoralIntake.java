package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Effector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

/**
 * Coral intake command that implements a 3-stage intake sequence:
 * 
 * Stage 1 - Passive Intake: No coral detected yet
 *   - Elevator moves to intake position
 *   - Intake wheels run at 20 RPS
 *   - Effector wheels run at 30 RPS (forward)
 * 
 * Stage 2 - Slow Intake: Coral detected by sensor
 *   - Intake wheels stop
 *   - Effector wheels slow to 20 RPS (forward) to gently pull coral in
 * 
 * Stage 3 - Reverse Lock: Coral passed sensor (no longer detected)
 *   - Effector reverses to -20 RPS to lock coral in place
 * 
 * Command completes when coral is detected again after the reverse phase,
 * indicating the coral has been successfully locked in place.
 */
public class CoralIntake extends Command {
    private final Intake m_intake;
    private final Effector m_effector;
    private final Elevator m_elevator;
    
    /** Tracks whether coral has been detected at least once during this intake */
    private boolean coralHasBeenSeen = false;
    /** Tracks whether the reverse phase has been executed */
    private boolean coralHasBeenReversed = false;

    /**
     * Creates a new CoralIntake command.
     * 
     * @param m_elevator Elevator subsystem to move to intake position
     * @param m_effector Effector subsystem to control scoring wheels
     * @param m_intake Intake subsystem to control funnel intake wheels
     */
    public CoralIntake(Elevator m_elevator, Effector m_effector, Intake m_intake) {
        addRequirements(m_intake, m_effector, m_elevator);
        this.m_elevator = m_elevator;
        this.m_effector = m_effector;
        this.m_intake = m_intake;
    }

    @Override
    public void initialize() {
        // Reset state machine flags
        coralHasBeenSeen = false;
        coralHasBeenReversed = false;
        
        // Move elevator to intake height
        m_elevator.toPosition(Constants.elevator.level.INTAKE);
    }

    @Override
    public void execute() {
        // State machine for 3-stage intake sequence
        if (!m_effector.isCoralDetected() && !coralHasBeenSeen) {
            // Stage 1: Passive intake - coral not yet detected
            // Run intake and effector at full speed to pull coral in
            m_intake.start(20);
            m_effector.start(30.0);
        }
        else if (m_effector.isCoralDetected() && !coralHasBeenReversed) {
            // Stage 2: Coral detected - slow down to gently pull it in
            coralHasBeenSeen = true;
            m_intake.stop();  // Stop ground intake
            m_effector.start(20);  // Slow down effector
        }
        else if (!m_effector.isCoralDetected() && coralHasBeenSeen) {
            // Stage 3: Coral passed sensor - reverse to lock it in place
            coralHasBeenReversed = true;
            m_effector.start(-20);  // Reverse effector to lock coral
        }
        // Note: If coral is detected again after reverse, command will finish
    }

    @Override
    public boolean isFinished() {
        // Command completes when coral is detected again after the reverse phase
        // This indicates the coral has been successfully locked in place
        return m_effector.isCoralDetected() && coralHasBeenSeen && coralHasBeenReversed;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all mechanisms
        m_intake.stop();
        m_effector.stop();
        
        // Lower elevator to L1 (bottom) position
        m_elevator.toPosition(Constants.elevator.level.L1);
    }
}
