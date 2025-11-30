package frc.robot.state;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotStateManager {
    private RobotState currentState = RobotState.DISABLED;
    private RobotState previousState = RobotState.DISABLED;
    private double lastStateChangeTime = Timer.getFPGATimestamp();

    public synchronized RobotState getState() {
        return currentState;
    }

    public synchronized RobotState getPreviousState() {
        return previousState;
    }

    public synchronized void setState(RobotState newState) {
        if (newState == currentState) {
            return;
        }
        previousState = currentState;
        currentState = newState;
        lastStateChangeTime = Timer.getFPGATimestamp();

        System.out.println("[RobotState] " + previousState + " -> " + currentState);

        SmartDashboard.putString("Robot/State", currentState.name());
        SmartDashboard.putString("Robot/PrevState", previousState.name());
    }

    /** Seconds since the last call to setState. */
    public synchronized double getTimeInState() {
        return Timer.getFPGATimestamp() - lastStateChangeTime;
    }
}
