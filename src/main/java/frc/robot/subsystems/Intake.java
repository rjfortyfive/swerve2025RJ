package frc.robot.subsystems;

import frc.robot.Constants;
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

public Intake() {
    //setting Intake motors to Brake mode
    intakeLeftFX.setNeutralMode(NeutralModeValue.Brake);
    intakeRightFX.setNeutralMode(NeutralModeValue.Brake);

    //Setting Right intake motor to follow Left intake motor
    intakeRightFX.setControl(new Follower(intakeLeftFX.getDeviceID(), true));

    //Current Limits Configuration for intake motors
    intakeLeftFX.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Constants.effector.EFFECTOR_STATOR_CURRENT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Constants.effector.EFFECTOR_SUPPLY_CURRENT)
        .withSupplyCurrentLimitEnable(true));

    intakeRightFX.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Constants.effector.EFFECTOR_STATOR_CURRENT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Constants.effector.EFFECTOR_SUPPLY_CURRENT)
        .withSupplyCurrentLimitEnable(true));
    
    //PID Configuration for intake motors
    intakeLeftFX.getConfigurator().apply( new Slot0Configs()
        .withKP(Constants.intake.P_INTAKE)
        .withKI(Constants.intake.I_INTAKE)
        .withKD(Constants.intake.D_INTAKE)
        .withKG(Constants.intake.G_INTAKE)
        .withKS(Constants.intake.S_INTAKE)
        .withKV(Constants.intake.V_INTAKE)
        .withKA(Constants.intake.A_INTAKE));

    intakeRightFX.getConfigurator().apply( new Slot0Configs()
        .withKP(Constants.intake.P_INTAKE)
        .withKI(Constants.intake.I_INTAKE)
        .withKD(Constants.intake.D_INTAKE)
        .withKG(Constants.intake.G_INTAKE)
        .withKS(Constants.intake.S_INTAKE)
        .withKV(Constants.intake.V_INTAKE)
        .withKA(Constants.intake.A_INTAKE));

    //Max Voltage configuration for intake motors
    intakeLeftFX.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE))
        .withPeakReverseVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE)));
            
    intakeRightFX.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE))
        .withPeakReverseVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE)));

}

public void stop() {
    
    intakeLeftFX.set(0);

}

public void start(double velocity) {
        
    intakeLeftFX.setControl(m_velocityVoltage.withVelocity(velocity * Constants.MASTER_SPEED_MULTIPLIER));

}
}