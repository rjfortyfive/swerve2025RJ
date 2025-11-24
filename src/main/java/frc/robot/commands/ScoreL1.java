package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Effector;
import frc.robot.subsystems.Elevator;

public class ScoreL1 extends Command {

    public ScoreL1(Elevator m_Elevator, Effector m_Effector) {

        addRequirements(m_Effector, m_Elevator);


    }
    
}
