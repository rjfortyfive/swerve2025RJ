package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Effector;
import frc.robot.subsystems.Elevator;

/**
 * Smart scoring command that automatically selects the appropriate scoring method
 * based on the current elevator position:
 * - L1: Asymmetric outtake (20.0, 6.0 velocities) - runs while button is held
 * - L2/L3/L4: Standard ScoreL4L3L2 command - runs until coral is scored
 */
public class SmartScore {
    
    // Tolerance for determining if elevator is at L1 (2 rotations)
    private static final double ELEVATOR_TOLERANCE = 2.0;
    
    /**
     * Creates a command that selects the appropriate scoring method based on elevator position.
     * This can be used with either onTrue (for L2/L3/L4) or whileTrue (for L1 asymmetric).
     * 
     * @param elevator The elevator subsystem to check position
     * @param effector The effector subsystem to control
     * @return A command that runs the appropriate scoring method
     */
    public static Command create(Elevator elevator, Effector effector) {
        return Commands.defer(() -> {
            double currentPos = elevator.getPosition();
            double l1Pos = Constants.elevator.level.L1;
            boolean isAtL1 = Math.abs(currentPos - l1Pos) < ELEVATOR_TOLERANCE;
            
            if (isAtL1) {
                // L1: Asymmetric outtake (runs while button is held, like A button)
                return new StartEndCommand(
                    () -> effector.start(20.0, 6.0),
                    () -> effector.stop(),
                    effector
                );
            } else {
                // L2/L3/L4: Standard scoring command (runs until coral is scored)
                return new ScoreL4L3L2(effector);
            }
        }, java.util.Set.of(elevator, effector));
    }
}

