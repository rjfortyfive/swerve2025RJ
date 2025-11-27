package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Effector;

public class ScoreL2L3 extends Command {
    private final Effector m_effector;

    public ScoreL2L3(Effector m_effector) {

        this.m_effector = m_effector;
        addRequirements(m_effector);

    }
    
    @Override
    public void execute() {

    }

    // @Override
    // public boolean isFinished() {
 
    // }

    @Override
    public void end(boolean interrupted) {
        m_effector.stop();
    }
}
