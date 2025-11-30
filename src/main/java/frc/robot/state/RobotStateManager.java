package frc.robot.state;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotStateManager {
    private RobotState currentState = RobotState.DISABLED;
    private RobotState previousState = RobotState.DISABLED;
    private double stateStart = Timer.getFPGATimestamp();

    public RobotState getState() {
        return currentState;
    }

    public void setState(RobotState s) {
        if (s == currentState) return;

        previousState = currentState;
        currentState = s;
        stateStart = Timer.getFPGATimestamp();

        SmartDashboard.putString("RobotState/Current", s.name());
        SmartDashboard.putString("RobotState/Previous", previousState.name());
    }

    public double timeInState() {
        return Timer.getFPGATimestamp() - stateStart;
    }
}
