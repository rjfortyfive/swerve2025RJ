package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Effector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.RobotState;

/**
 * State-aware version of CoralIntake that manages state transitions
 */
public class StateAwareCoralIntake {
    
    /**
     * Creates a command sequence that transitions to INTAKING state, runs intake, then returns to IDLE
     */
    public static Command create(StateManager stateManager, Elevator elevator, Effector effector, Intake intake) {
        return Commands.sequence(
            // Transition to INTAKING state
            new SetStateCommand(stateManager, RobotState.INTAKING),
            
            // Run the actual intake command
            new CoralIntake(elevator, effector, intake),
            
            // Return to IDLE when done
            new SetStateCommand(stateManager, RobotState.IDLE)
        );
    }
}

