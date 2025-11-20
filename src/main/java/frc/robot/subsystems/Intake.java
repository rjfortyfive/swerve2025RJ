package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake extends SubsystemBase {

    private static TalonFX intakeLeftFX = new TalonFX(Constants.intake.INTAKE_LEFT_ID);
    private static TalonFX intakeRightFX = new TalonFX(Constants.intake.INTAKE_RIGHT_ID);

public Intake() {
    
    intakeLeftFX.setControl(new Follower(intakeRightFX.getDeviceID(), true));

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

    intakeLeftFX.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE))
        .withPeakReverseVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE)));
            
    intakeRightFX.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE))
        .withPeakReverseVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE)));

}

}