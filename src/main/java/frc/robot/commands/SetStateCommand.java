package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.RobotState;

/**
 * Command to transition the robot to a specific state.
 * This can be used in button bindings or as part of command sequences.
 */
public class SetStateCommand extends Command {
    private final StateManager m_stateManager;
    private final RobotState m_targetState;
    private final boolean m_force;
    
    /**
     * Creates a command to transition to a state
     * @param stateManager The state manager subsystem
     * @param targetState The state to transition to
     */
    public SetStateCommand(StateManager stateManager, RobotState targetState) {
        this(stateManager, targetState, false);
    }
    
    /**
     * Creates a command to transition to a state
     * @param stateManager The state manager subsystem
     * @param targetState The state to transition to
     * @param force If true, forces the transition (bypasses validation)
     */
    public SetStateCommand(StateManager stateManager, RobotState targetState, boolean force) {
        m_stateManager = stateManager;
        m_targetState = targetState;
        m_force = force;
        
        // State manager doesn't need to be a requirement - it's just a data holder
    }
    
    @Override
    public void initialize() {
        if (m_force) {
            m_stateManager.forceState(m_targetState);
        } else {
            boolean success = m_stateManager.setState(m_targetState);
            if (!success) {
                System.out.println("Warning: Invalid state transition to " + m_targetState);
            }
        }
    }
    
    @Override
    public boolean isFinished() {
        // State transitions are instantaneous
        return true;
    }
}

