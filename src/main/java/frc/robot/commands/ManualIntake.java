package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Effector;

import edu.wpi.first.wpilibj2.command.Command;


public class ManualIntake extends Command {

    private final Intake intake;
    private final Effector effector;

    public ManualIntake(Intake intake, Effector effector) {
        this.intake = intake;
        this.effector = effector;

        addRequirements(intake, effector);
    }

    @Override
    public void execute() {

        // Intake wheels (your Intake class already uses only left and follows to the right)
        intake.start(40);

        // Effector wheels (your Effector class runs both left & right same direction)
        effector.start(40);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        effector.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // manual command, runs until button released
    }
}
