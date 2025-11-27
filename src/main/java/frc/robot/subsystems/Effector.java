package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;


public class Effector extends SubsystemBase {
    private static LaserCan intakeSensor = new LaserCan(2);

    private static TalonFX effectorLeftFX = new TalonFX(Constants.effector.EFFECTOR_LEFT_FX_ID);
    private static TalonFX effectorRightFX = new TalonFX(Constants.effector.EFFECTOR_RIGHT_FX_ID);

    private static SparkMax algaeMotor = new SparkMax(1, MotorType.kBrushed);

    private final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);

    private final static PositionVoltage m_positionVoltage = new PositionVoltage(0);

    public Effector() {
        SparkMaxConfig algaeConfig = new SparkMaxConfig();

        effectorLeftFX.setNeutralMode(NeutralModeValue.Brake);
        effectorRightFX.setNeutralMode(NeutralModeValue.Brake);

        algaeConfig.idleMode(IdleMode.kBrake);

        effectorRightFX.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive));

        effectorLeftFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.effector.EFFECTOR_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.effector.EFFECTOR_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));

        effectorRightFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.effector.EFFECTOR_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.effector.EFFECTOR_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));
        
        algaeConfig.smartCurrentLimit(20);

        effectorLeftFX.getConfigurator().apply( new Slot0Configs()
            .withKP(Constants.effector.P_EFFECTOR)
            .withKI(Constants.effector.I_EFFECTOR)
            .withKD(Constants.effector.D_EFFECTOR)
            .withKG(Constants.effector.G_EFFECTOR)
            .withKS(Constants.effector.S_EFFECTOR)
            .withKV(Constants.effector.V_EFFECTOR)
            .withKA(Constants.effector.A_EFFECTOR));
            
        effectorRightFX.getConfigurator().apply( new Slot0Configs()
            .withKP(Constants.effector.P_EFFECTOR)
            .withKI(Constants.effector.I_EFFECTOR)
            .withKD(Constants.effector.D_EFFECTOR)
            .withKG(Constants.effector.G_EFFECTOR)
            .withKS(Constants.effector.S_EFFECTOR)
            .withKV(Constants.effector.V_EFFECTOR)
            .withKA(Constants.effector.A_EFFECTOR));
        
        effectorLeftFX.getConfigurator().apply(new VoltageConfigs()
            .withPeakForwardVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE)));
            
        effectorRightFX.getConfigurator().apply(new VoltageConfigs()
            .withPeakForwardVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE)));
        
        effectorLeftFX.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(30 * Constants.MASTER_SPEED_MULTIPLIER))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100 * Constants.MASTER_SPEED_MULTIPLIER)));

        effectorRightFX.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(30 * Constants.MASTER_SPEED_MULTIPLIER))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100 * Constants.MASTER_SPEED_MULTIPLIER)));
    
        AbsoluteEncoderConfig algaeEncoder = algaeConfig.absoluteEncoder;
        algaeEncoder.positionConversionFactor(1.0);
        algaeEncoder.velocityConversionFactor(1.0);
        algaeEncoder.zeroOffset(0.0);
        algaeMotor.configure(
            algaeConfig,
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters
        );

    }

    @Override
    public void periodic() {

    }

    public double getLeftPosition() {
        return effectorLeftFX.getPosition().getValueAsDouble();
    }

    public double getRightPosition() {
        return effectorRightFX.getPosition().getValueAsDouble();
    }

    public void moveToPositions(double leftPos, double rightPos) {
        effectorLeftFX.setControl(m_positionVoltage.withPosition(leftPos));
        effectorRightFX.setControl(m_positionVoltage.withPosition(rightPos));
    }
// Check if both motors are at position
    public boolean coralAtPosition(double leftTarget, double rightTarget) {
        double leftErr  = Math.abs(getLeftPosition()  - leftTarget);
        double rightErr = Math.abs(getRightPosition() - rightTarget);
        return leftErr < 0.02 && rightErr < 0.02;   // tweak threshold if needed
}

    public void stop() {
        effectorLeftFX.set(0);
        effectorRightFX.set(0);

    }

    public void start(double velocity) {

        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(velocity * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(velocity * Constants.MASTER_SPEED_MULTIPLIER));

    }

    public void start(double velocityLeft, double velocityRight) {
        // Turn on intake and effector wheels
        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(velocityLeft * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(velocityRight * Constants.MASTER_SPEED_MULTIPLIER));

    }

    public void startLock() {
        // Turn on intake
        double leftCurrentPos = effectorLeftFX.getPosition().getValueAsDouble();
        double rightCurrentPos = effectorRightFX.getPosition().getValueAsDouble();

        double leftTarget = leftCurrentPos + Constants.intake.lockRotations;
        double rightTarget = rightCurrentPos - Constants.intake.lockRotations; 

        effectorLeftFX.setControl(m_positionVoltage.withPosition(leftTarget));
        effectorRightFX.setControl(m_positionVoltage.withPosition(rightTarget));
    }

    public void reverseLock() {

        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(-15 * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(15 * Constants.MASTER_SPEED_MULTIPLIER));
    }

    // public void startOut(double velocityLeft, double velocityRight) {
    //     // Turn on intake
    //     effectorLeftFX.setControl(m_velocityVoltage.withVelocity(velocityLeft * Constants.MASTER_SPEED_MULTIPLIER));
    //     effectorRightFX.setControl(m_velocityVoltage.withVelocity(-velocityRight * Constants.MASTER_SPEED_MULTIPLIER));

    // }

    public boolean isCoralDetected() {
        return intakeSensor.getMeasurement().distance_mm < 10;
    }

    public boolean isCoralNotDetected() {
        return intakeSensor.getMeasurement().distance_mm > 10;
    }

    public void manualEffectorOut(double velocityLeft, Double velocityRight) {

        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(velocityLeft * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(-velocityRight * Constants.MASTER_SPEED_MULTIPLIER));

    }

    public void algaeEffectorUp(double percent) {
        algaeMotor.set(-percent);
    }

    public void algaeEffectorDown(double percent) {
        algaeMotor.set(percent);

    }
}
