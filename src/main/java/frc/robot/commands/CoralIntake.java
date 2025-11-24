package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Effector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class CoralIntake extends Command {
    private final Intake m_intake;
    private final Effector m_effector;
    private final Elevator m_elevator;
    private boolean coralHasBeenSeen = false;
    private boolean coralHasBeenReversed = false;


    public CoralIntake(Elevator m_elevator, Effector m_effector, Intake m_intake) {

        addRequirements(m_intake, m_effector, m_elevator);
        this.m_elevator = m_elevator;
        this.m_effector = m_effector;
        this.m_intake = m_intake;

    }

    @Override
    public void initialize() {
        coralHasBeenSeen = false;
        coralHasBeenReversed = false;
        m_elevator.toPosition(Constants.elevator.level.intake);
    }

    @Override
    public void execute() {
        if (!m_effector.isCoralDetected() && !coralHasBeenSeen) // no coral, never been seen = passive intake
        {
            m_intake.startIntake();
            m_effector.startIntake();
        }
        else if (m_effector.isCoralDetected() && !coralHasBeenReversed)  // yes coral, and not reversed yet = slow intake
        {
            coralHasBeenSeen = true;
            m_intake.stopIntake();
            m_effector.startLock();

        }
        else if (!m_effector.isCoralDetected() && coralHasBeenSeen) // no coral, was seen = reverse
        {
            coralHasBeenReversed = true;
            m_effector.reverseLock();
        }

    }

    @Override
    public boolean isFinished() {
        return m_effector.isCoralDetected() && coralHasBeenSeen && coralHasBeenReversed;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntake();
        m_effector.stopIntake();
        m_elevator.toPosition(Constants.elevator.level.L1);
    }
}
