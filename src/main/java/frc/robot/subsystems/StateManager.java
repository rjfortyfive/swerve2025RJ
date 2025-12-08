package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * StateManager manages the robot's operational state machine.
 * This works alongside the command-based framework to provide state-based control.
 */
public class StateManager extends SubsystemBase {
    
    /**
     * Robot operational states
     */
    public enum RobotState {
        IDLE,              // Robot is idle, ready for commands
        INTAKING,          // Intaking coral from station
        SCORING_L1,        // Scoring at level 1
        SCORING_L2,        // Scoring at level 2
        SCORING_L3,        // Scoring at level 3
        SCORING_L4,        // Scoring at level 4
        CLIMBING,          // Climbing/hanging
        MANUAL             // Manual control mode (bypasses state checks)
    }
    
    private RobotState currentState = RobotState.IDLE;
    private RobotState previousState = RobotState.IDLE;
    
    public StateManager() {
        // Initialize state
        setState(RobotState.IDLE);
    }
    
    @Override
    public void periodic() {
        // Publish current state to SmartDashboard
        SmartDashboard.putString("StateManager/CurrentState", currentState.toString());
        SmartDashboard.putString("StateManager/PreviousState", previousState.toString());
    }
    
    /**
     * Gets the current robot state
     * @return Current RobotState
     */
    public RobotState getState() {
        return currentState;
    }
    
    /**
     * Gets the previous robot state
     * @return Previous RobotState
     */
    public RobotState getPreviousState() {
        return previousState;
    }
    
    /**
     * Sets the robot state (with validation)
     * @param newState The state to transition to
     * @return true if transition was successful, false if invalid
     */
    public boolean setState(RobotState newState) {
        if (isValidTransition(currentState, newState)) {
            previousState = currentState;
            currentState = newState;
            return true;
        }
        return false;
    }
    
    /**
     * Forces a state transition (bypasses validation)
     * Use with caution - only for emergency overrides
     * @param newState The state to force transition to
     */
    public void forceState(RobotState newState) {
        previousState = currentState;
        currentState = newState;
    }
    
    /**
     * Checks if a state transition is valid
     * @param fromState Current state
     * @param toState Desired state
     * @return true if transition is allowed
     */
    private boolean isValidTransition(RobotState fromState, RobotState toState) {
        // Can always go to IDLE or MANUAL
        if (toState == RobotState.IDLE || toState == RobotState.MANUAL) {
            return true;
        }
        
        // Can always transition from IDLE or MANUAL
        if (fromState == RobotState.IDLE || fromState == RobotState.MANUAL) {
            return true;
        }
        
        // Can transition between scoring levels
        if (isScoringState(fromState) && isScoringState(toState)) {
            return true;
        }
        
        // Can transition from INTAKING to IDLE or scoring states
        if (fromState == RobotState.INTAKING) {
            return toState == RobotState.IDLE || isScoringState(toState);
        }
        
        // Can transition from scoring states to IDLE or other scoring states
        if (isScoringState(fromState)) {
            return toState == RobotState.IDLE || isScoringState(toState);
        }
        
        // Can transition from CLIMBING to IDLE
        if (fromState == RobotState.CLIMBING) {
            return toState == RobotState.IDLE;
        }
        
        // Default: disallow transition
        return false;
    }
    
    /**
     * Checks if a state is a scoring state
     * @param state State to check
     * @return true if state is a scoring state
     */
    public boolean isScoringState(RobotState state) {
        return state == RobotState.SCORING_L1 ||
               state == RobotState.SCORING_L2 ||
               state == RobotState.SCORING_L3 ||
               state == RobotState.SCORING_L4;
    }
    
    /**
     * Checks if robot is currently in a scoring state
     * @return true if current state is a scoring state
     */
    public boolean isScoring() {
        return isScoringState(currentState);
    }
    
    /**
     * Checks if robot is currently intaking
     * @return true if current state is INTAKING
     */
    public boolean isIntaking() {
        return currentState == RobotState.INTAKING;
    }
    
    /**
     * Checks if robot is currently climbing
     * @return true if current state is CLIMBING
     */
    public boolean isClimbing() {
        return currentState == RobotState.CLIMBING;
    }
    
    /**
     * Checks if robot is in manual mode
     * @return true if current state is MANUAL
     */
    public boolean isManual() {
        return currentState == RobotState.MANUAL;
    }
    
    /**
     * Resets state to IDLE
     */
    public void resetToIdle() {
        setState(RobotState.IDLE);
    }
}

