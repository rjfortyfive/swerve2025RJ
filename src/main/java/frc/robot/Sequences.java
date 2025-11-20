package frc.robot;

import frc.robot.subsystems.*;

public class Sequences {
    private final Elevator m_Elevator = new Elevator();
    public void removeL2Algae() {
        m_Elevator.toPosition(Constants.elevator.algaeLevel.L2);
        Effector.algaeEffectorUp(0.2);
        m_Elevator.toPosition(Constants.elevator.algaeLevel.L2 - 3);
    }

    public void removeL3Algae() {
        m_Elevator.toPosition(Constants.elevator.algaeLevel.L3);
        Effector.algaeEffectorUp(0.2);
        m_Elevator.toPosition(Constants.elevator.algaeLevel.L3 - 3);
    }

    public void scoreL1Coral() {
        m_Elevator.toPosition(Constants.elevator.level.L1);
        Effector.asymmetricalOuttake(null, null);
    }
}