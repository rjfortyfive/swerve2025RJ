package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Effector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.RobotState;

/**
 * State-aware scoring commands that manage state transitions
 */
public class StateAwareScore {
    
    /**
     * Creates a command sequence for scoring at a specific level
     * @param stateManager The state manager
     * @param elevator The elevator subsystem
     * @param effector The effector subsystem
     * @param level The scoring level (1-4)
     * @return Command sequence that transitions state, scores, then returns to IDLE
     */
    public static Command create(StateManager stateManager, Elevator elevator, Effector effector, int level) {
        RobotState scoringState;
        switch (level) {
            case 1:
                scoringState = RobotState.SCORING_L1;
                break;
            case 2:
                scoringState = RobotState.SCORING_L2;
                break;
            case 3:
                scoringState = RobotState.SCORING_L3;
                break;
            case 4:
                scoringState = RobotState.SCORING_L4;
                break;
            default:
                scoringState = RobotState.IDLE;
        }
        
        // For L1, use asymmetric outtake; for L2/L3/L4, use ScoreL4L3L2
        Command scoringCommand;
        if (level == 1) {
            // L1: Asymmetric outtake command
            scoringCommand = new ScoreL1Asymmetric(effector);
        } else {
            // L2, L3, L4: Use ScoreL4L3L2 command
            scoringCommand = new ScoreL4L3L2(effector);
        }
        
        return Commands.sequence(
            // Transition to scoring state
            new SetStateCommand(stateManager, scoringState),
            
            // Run the appropriate scoring command
            scoringCommand,
            
            // Return to IDLE when done
            new SetStateCommand(stateManager, RobotState.IDLE)
        );
    }
    
    /**
     * Creates a command sequence for scoring that automatically determines the level
     * based on the current elevator position
     * @param stateManager The state manager
     * @param elevator The elevator subsystem
     * @param effector The effector subsystem
     * @return Command sequence that transitions state, scores, then returns to IDLE
     */
    public static Command createAutoLevel(StateManager stateManager, Elevator elevator, Effector effector) {
        return Commands.sequence(
            // Determine level from elevator position and transition to scoring state
            Commands.runOnce(() -> {
                double currentPos = elevator.getPosition();
                RobotState scoringState = RobotState.IDLE;
                
                // Determine which level based on elevator position (with tolerance)
                double tolerance = 2.0; // Allow 2 rotations of tolerance
                if (Math.abs(currentPos - Constants.elevator.level.L4) < tolerance) {
                    scoringState = RobotState.SCORING_L4;
                } else if (Math.abs(currentPos - Constants.elevator.level.L3) < tolerance) {
                    scoringState = RobotState.SCORING_L3;
                } else if (Math.abs(currentPos - Constants.elevator.level.L2) < tolerance) {
                    scoringState = RobotState.SCORING_L2;
                } else if (Math.abs(currentPos - Constants.elevator.level.L1) < tolerance) {
                    scoringState = RobotState.SCORING_L1;
                }
                
                // Only transition if we determined a valid scoring state
                if (scoringState != RobotState.IDLE) {
                    stateManager.setState(scoringState);
                }
            }),
            
            // Run the appropriate scoring command based on detected level
            Commands.defer(() -> {
                double currentPos = elevator.getPosition();
                double tolerance = 2.0;
                
                // Determine level again (since we're in a deferred command)
                if (Math.abs(currentPos - Constants.elevator.level.L1) < tolerance) {
                    // L1: Asymmetric outtake
                    return new ScoreL1Asymmetric(effector);
                } else {
                    // L2, L3, L4: Use ScoreL4L3L2 command
                    return new ScoreL4L3L2(effector);
                }
            }, Set.of(elevator, effector)),
            
            // Return to IDLE when done
            new SetStateCommand(stateManager, RobotState.IDLE)
        );
    }
}

