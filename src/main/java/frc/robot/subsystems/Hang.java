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
import frc.robot.state.*;

public class Hang extends SubsystemBase {
    private final RobotStateManager sm;

    public static TalonFX hangFX = new TalonFX(Constants.hang.HANG_FX_ID);
    public final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

    public Hang(RobotStateManager sm) {
        this.sm = sm;

        // 🔒 Original config
        hangFX.setNeutralMode(NeutralModeValue.Brake);

        hangFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.hang.HANG_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.hang.HANG_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));

        hangFX.getConfigurator().apply( new Slot0Configs()
            .withKP(Constants.hang.P_HANG)
            .withKI(Constants.hang.I_HANG)
            .withKD(Constants.hang.D_HANG)
            .withKS(Constants.hang.S_HANG));

        hangFX.getConfigurator().apply(new VoltageConfigs()
            .withPeakForwardVoltage(Volts.of(Constants.hang.HANG_PEAK_FORWARD_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(Constants.hang.HANG_PEAK_REVERSE_VOLTAGE)));
    }

    @Override
    public void periodic() {
        if (sm.getState() == RobotState.CLIMB) {
            start(Constants.hang.CLIMB_VELOCITY);

            // Auto-timeout to IDLE after 3 seconds
            if (sm.timeInState() > 3.0) {
                stop();
                sm.setState(RobotState.IDLE);
            }
        } else {
            stop();
        }
    }

    public void stop() {
        hangFX.set(0);
    } 

    public void start(double velocity) {
        // Note: your original code ignored the velocity parameter, I keep that behavior:
        hangFX.setControl(m_velocityVoltage.withVelocity(100 * Constants.MASTER_SPEED_MULTIPLIER));
    }
}
