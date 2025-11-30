package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.state.RobotState;
import frc.robot.state.RobotStateManager;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake extends SubsystemBase {

    private static TalonFX intakeLeftFX = new TalonFX(Constants.intake.INTAKE_LEFT_FX_ID);
    private static TalonFX intakeRightFX = new TalonFX(Constants.intake.INTAKE_RIGHT_FX_ID);

    private final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);

    private final RobotStateManager stateManager;

    public Intake(RobotStateManager stateManager) {
        this.stateManager = stateManager;

        // existing CTRE setup & configs…
    }

    @Override
    public void periodic() {
        RobotState s = stateManager.getState();

        switch (s) {
            case INTAKE_CORAL:
                start(Constants.intake.CORAL_INTAKE_VEL);
                break;
            case INTAKE_ALGAE:
                start(Constants.intake.ALGAE_HELPER_VEL); // if you want intake assisting algae
                break;
            default:
                stop();
                break;
        }
    }

    public void stop() {
        intakeLeftFX.set(0);
    }

    public void start(double velocity) {
        intakeLeftFX.setControl(m_velocityVoltage.withVelocity(velocity * Constants.MASTER_SPEED_MULTIPLIER));
    }
}
