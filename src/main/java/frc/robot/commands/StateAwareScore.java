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
     * @throws IllegalArgumentException if level is not 1-4
     */
    public static Command create(StateManager stateManager, Elevator elevator, Effector effector, int level) {
        // Validate level - reject invalid levels to prevent state mismatch
        if (level < 1 || level > 4) {
            System.err.println("Error: Invalid scoring level " + level + ". Level must be 1-4.");
            // Return a no-op command that logs the error and does nothing
            return Commands.print("Invalid scoring level: " + level + ". Command cancelled.");
        }
        
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
                // This should never be reached due to validation above, but keep for safety
                System.err.println("Error: Unexpected level in switch: " + level);
                return Commands.print("Unexpected scoring level: " + level + ". Command cancelled.");
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
        // Store detected level to ensure consistency between state and command selection
        // Use array to allow modification within lambda
        final int[] detectedLevel = {0};
        
        return Commands.sequence(
            // Wait for elevator to be within tolerance of a level position
            // This prevents detecting intermediate positions when elevator is mid-motion
            Commands.waitUntil(() -> {
                double currentPos = elevator.getPosition();
                double tolerance = 2.0; // Allow 2 rotations of tolerance
                // Check if elevator is within tolerance of any level
                return Math.abs(currentPos - Constants.elevator.level.L4) < tolerance ||
                       Math.abs(currentPos - Constants.elevator.level.L3) < tolerance ||
                       Math.abs(currentPos - Constants.elevator.level.L2) < tolerance ||
                       Math.abs(currentPos - Constants.elevator.level.L1) < tolerance;
            }).withTimeout(2.0), // Timeout after 2 seconds to prevent infinite waiting
            
            // Determine level from elevator position and transition to scoring state
            // Store the detected level to ensure consistency with command selection
            Commands.runOnce(() -> {
                double currentPos = elevator.getPosition();
                RobotState scoringState = RobotState.IDLE;
                
                // Determine which level based on elevator position (with tolerance)
                double tolerance = 2.0; // Allow 2 rotations of tolerance
                if (Math.abs(currentPos - Constants.elevator.level.L4) < tolerance) {
                    scoringState = RobotState.SCORING_L4;
                    detectedLevel[0] = 4;
                } else if (Math.abs(currentPos - Constants.elevator.level.L3) < tolerance) {
                    scoringState = RobotState.SCORING_L3;
                    detectedLevel[0] = 3;
                } else if (Math.abs(currentPos - Constants.elevator.level.L2) < tolerance) {
                    scoringState = RobotState.SCORING_L2;
                    detectedLevel[0] = 2;
                } else if (Math.abs(currentPos - Constants.elevator.level.L1) < tolerance) {
                    scoringState = RobotState.SCORING_L1;
                    detectedLevel[0] = 1;
                } else {
                    // No level matches within tolerance - find closest level as fallback
                    // This ensures we always have a valid state when scoring
                    double distL1 = Math.abs(currentPos - Constants.elevator.level.L1);
                    double distL2 = Math.abs(currentPos - Constants.elevator.level.L2);
                    double distL3 = Math.abs(currentPos - Constants.elevator.level.L3);
                    double distL4 = Math.abs(currentPos - Constants.elevator.level.L4);
                    
                    double minDist = Math.min(Math.min(distL1, distL2), Math.min(distL3, distL4));
                    
                    // Use tolerance-based comparison to handle floating-point precision issues
                    double comparisonTolerance = 0.001; // Small tolerance for floating-point comparison
                    if (Math.abs(minDist - distL1) < comparisonTolerance) {
                        scoringState = RobotState.SCORING_L1;
                        detectedLevel[0] = 1;
                    } else if (Math.abs(minDist - distL2) < comparisonTolerance) {
                        scoringState = RobotState.SCORING_L2;
                        detectedLevel[0] = 2;
                    } else if (Math.abs(minDist - distL3) < comparisonTolerance) {
                        scoringState = RobotState.SCORING_L3;
                        detectedLevel[0] = 3;
                    } else {
                        scoringState = RobotState.SCORING_L4;
                        detectedLevel[0] = 4;
                    }
                }
                
                // Always transition to the determined scoring state
                // This ensures state accurately reflects robot operation
                stateManager.setState(scoringState);
            }),
            
            // Run the appropriate scoring command based on detected level
            // Use the stored level from runOnce to ensure consistency with state
            Commands.defer(() -> {
                // Use the level detected in runOnce to ensure state-command consistency
                // This prevents mismatch if elevator position changes between state detection and command selection
                int level = detectedLevel[0];
                
                // Select command based on stored level - matches state detection
                if (level == 1) {
                    // L1: Asymmetric outtake
                    return new ScoreL1Asymmetric(effector);
                } else {
                    // L2, L3, L4: Use ScoreL4L3L2 command (works for all three levels)
                    return new ScoreL4L3L2(effector);
                }
            }, Set.of(elevator, effector)),
            
            // Return to IDLE when done
            new SetStateCommand(stateManager, RobotState.IDLE)
        );
    }
}

