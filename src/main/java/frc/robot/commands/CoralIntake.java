package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Effector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import com.ctre.phoenix6.Utils;

public class CoralIntake extends Command {
    private final Intake m_intake;
    private final Effector m_effector;
    private final Elevator m_elevator;
    private boolean coralHasBeenSeen = false;
    private boolean coralHasBeenReversed = false;
    private double commandStartTime = 0.0;
    private static final double COMMAND_TIMEOUT = 5.0; // Timeout in seconds for entire intake sequence
    private static final double SIM_COMMAND_TIMEOUT = 3.0; // Shorter timeout for simulation


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
        commandStartTime = Timer.getFPGATimestamp();
        m_elevator.toPosition(Constants.elevator.level.INTAKE);
    }

    @Override
    public void execute() {
        if (!m_effector.isCoralDetected() && !coralHasBeenSeen) // no coral, never been seen = passive intake
        {
            m_intake.start(20);
            m_effector.start(30.0);
        }
        else if (m_effector.isCoralDetected() && !coralHasBeenReversed)  // yes coral, and not reversed yet = slow intake
        {
            coralHasBeenSeen = true;
            m_intake.stop();
            m_effector.start(20);

        }
        else if (!m_effector.isCoralDetected() && coralHasBeenSeen) // no coral, was seen = reverse
        {
            coralHasBeenReversed = true;
            m_effector.start(-20);
        }

    }

    @Override
    public boolean isFinished() {
        // Normal completion: coral detected after reverse
        if (m_effector.isCoralDetected() && coralHasBeenSeen && coralHasBeenReversed) {
            return true;
        }
        
        // Timeout check - prevent getting stuck
        double timeout = Utils.isSimulation() ? SIM_COMMAND_TIMEOUT : COMMAND_TIMEOUT;
        if ((Timer.getFPGATimestamp() - commandStartTime) >= timeout) {
            return true; // Timeout reached, finish command
        }
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        m_effector.stop();
        m_elevator.toPosition(Constants.elevator.level.L1);
    }
}
