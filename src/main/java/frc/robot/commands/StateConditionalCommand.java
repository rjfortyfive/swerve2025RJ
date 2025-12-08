package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.RobotState;

/**
 * Utility class for creating commands that only run if the robot is in a specific state
 */
public class StateConditionalCommand {
    
    /**
     * Creates a command that only runs if the robot is in the specified state
     * @param stateManager The state manager
     * @param requiredState The state required for the command to run
     * @param command The command to run if state matches
     * @param elseCommand The command to run if state doesn't match (can be null)
     * @return A ConditionalCommand that checks state before running
     */
    public static Command create(StateManager stateManager, RobotState requiredState, 
                                  Command command, Command elseCommand) {
        return new ConditionalCommand(
            command,
            elseCommand != null ? elseCommand : none(),
            () -> stateManager.getState() == requiredState
        );
    }
    
    /**
     * Creates a command that only runs if the robot is in any of the specified states
     * @param stateManager The state manager
     * @param allowedStates The states that allow the command to run
     * @param command The command to run if state matches
     * @param elseCommand The command to run if state doesn't match (can be null)
     * @return A ConditionalCommand that checks state before running
     */
    public static Command create(StateManager stateManager, RobotState[] allowedStates,
                                  Command command, Command elseCommand) {
        return new ConditionalCommand(
            command,
            elseCommand != null ? elseCommand : none(),
            () -> {
                RobotState current = stateManager.getState();
                for (RobotState state : allowedStates) {
                    if (current == state) return true;
                }
                return false;
            }
        );
    }
}

