package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Effector;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual intake command for manually controlling intake and effector.
 * 
 * This command runs both intake and effector at 40 RPS continuously while active.
 * Unlike CoralIntake, this command does not implement any state machine or
 * sensor-based logic - it simply runs the mechanisms at a fixed speed.
 * 
 * The command runs until cancelled (typically when a button is released).
 */
public class ManualIntake extends Command {

    private final Intake intake;
    private final Effector effector;

    /**
     * Creates a new ManualIntake command.
     * 
     * @param intake Intake subsystem to control ground intake wheels
     * @param effector Effector subsystem to control scoring wheels
     */
    public ManualIntake(Intake intake, Effector effector) {
        this.intake = intake;
        this.effector = effector;

        addRequirements(intake, effector);
    }

    @Override
    public void execute() {
        // Run intake wheels at 40 RPS
        // Note: Intake subsystem uses left motor as master, right motor follows
        intake.start(40);

        // Run effector wheels at 40 RPS (both left and right forward)
        effector.start(40);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop both mechanisms when command ends
        intake.stop();
        effector.stop();
    }

    @Override
    public boolean isFinished() {
        // Manual command - runs until button is released (command is cancelled)
        return false;
    }
}
