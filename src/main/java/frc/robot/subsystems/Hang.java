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
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.hang;

public class Hang extends SubsystemBase{
    private final TalonFX hangFX = new TalonFX(CanIDs.HANG_FX_ID);
    private final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

public Hang() {
    //setting Hang motor to Brake mode
    hangFX.setNeutralMode(NeutralModeValue.Brake);

    //Current Limits Configuration for hang motor
    hangFX.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(hang.HANG_STATOR_CURRENT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(hang.HANG_SUPPLY_CURRENT)
        .withSupplyCurrentLimitEnable(true));

    //PID Configuration for hang motor
    hangFX.getConfigurator().apply( new Slot0Configs()
        .withKP(hang.P_HANG)
        .withKI(hang.I_HANG)
        .withKD(hang.D_HANG)
        .withKS(hang.S_HANG));

    //Max Voltage configuration for hang motor
    hangFX.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(Volts.of(hang.HANG_PEAK_FORWARD_VOLTAGE))
        .withPeakReverseVoltage(Volts.of(-hang.HANG_PEAK_REVERSE_VOLTAGE)));
}    

    public void stop() {
        hangFX.set(0);
    } 

    public void start(double velocity) {
        hangFX.setControl(m_velocityVoltage.withVelocity(velocity * Constants.MASTER_SPEED_MULTIPLIER));
    }
}
