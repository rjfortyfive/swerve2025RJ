package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.state.RobotState;
import frc.robot.state.RobotStateManager;

public class Hang extends SubsystemBase {
    public static TalonFX hangFX = new TalonFX(Constants.hang.HANG_FX_ID);
    public final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

    private final RobotStateManager stateManager;

    public Hang(RobotStateManager stateManager) {
        this.stateManager = stateManager;

        // your existing config code…
    }

    @Override
    public void periodic() {
        if (stateManager.getState() == RobotState.CLIMB) {
            // climb up
            start(Constants.hang.CLIMB_VELOCITY);

            // auto-transition after N seconds (or add an encoder check)
            if (stateManager.getTimeInState() > 3.0) {
                stop();
                stateManager.setState(RobotState.IDLE);
            }
        } else {
            stop();
        }
    }

    public void stop() {
        hangFX.set(0);
    }

    public void start(double velocity) {
        hangFX.setControl(m_velocityVoltage.withVelocity(velocity * Constants.MASTER_SPEED_MULTIPLIER));
    }
}
