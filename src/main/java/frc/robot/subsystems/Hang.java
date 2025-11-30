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

public class Hang extends SubsystemBase{
    public static TalonFX hangFX = new TalonFX(Constants.hang.HANG_FX_ID);
    public final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

public Hang() {
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

    public void stop() {
        hangFX.set(0);
    } 

    public void start(double velocity) {
        hangFX.setControl(m_velocityVoltage.withVelocity(100 * Constants.MASTER_SPEED_MULTIPLIER));
    }
}
