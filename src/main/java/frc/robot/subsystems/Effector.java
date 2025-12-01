package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.effector;
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

    private static TalonFX effectorLeftFX = new TalonFX(CanIDs.EFFECTOR_LEFT_FX_ID);
    private static TalonFX effectorRightFX = new TalonFX(CanIDs.EFFECTOR_RIGHT_FX_ID);

    private static SparkMax algaeMotor = new SparkMax(1, MotorType.kBrushed);

    private final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);

    private final static PositionVoltage m_positionVoltage = new PositionVoltage(0);

    public Effector() {
        //algae Spark Max Configuration
        SparkMaxConfig algaeConfig = new SparkMaxConfig();

        //effector motor set to Brake Motor
        effectorLeftFX.setNeutralMode(NeutralModeValue.Brake);
        effectorRightFX.setNeutralMode(NeutralModeValue.Brake);

        //alage motor set to brake mode
        algaeConfig.idleMode(IdleMode.kBrake);

        //Setting Right effector motor to invert
        effectorRightFX.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive));

        //Current Limits Configuration for effector motors
        effectorLeftFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.effector.EFFECTOR_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(effector.EFFECTOR_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));

        effectorRightFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(effector.EFFECTOR_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(effector.EFFECTOR_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));
        
        //Current Limits Configuration for algae motor
        algaeConfig.smartCurrentLimit(20);

        //PID Configurations for effector motors
        effectorLeftFX.getConfigurator().apply( new Slot0Configs()
            .withKP(effector.P_EFFECTOR)
            .withKI(effector.I_EFFECTOR)
            .withKD(effector.D_EFFECTOR)
            .withKG(effector.G_EFFECTOR)
            .withKS(effector.S_EFFECTOR)
            .withKV(effector.V_EFFECTOR)
            .withKA(effector.A_EFFECTOR));
            
        effectorRightFX.getConfigurator().apply( new Slot0Configs()
            .withKP(effector.P_EFFECTOR)
            .withKI(effector.I_EFFECTOR)
            .withKD(effector.D_EFFECTOR)
            .withKG(effector.G_EFFECTOR)
            .withKS(effector.S_EFFECTOR)
            .withKV(effector.V_EFFECTOR)
            .withKA(effector.A_EFFECTOR));
        
        //Voltage Configurations for effector motors
        effectorLeftFX.getConfigurator().apply(new VoltageConfigs()
            .withPeakForwardVoltage(Volts.of(effector.EFFECTOR_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(-effector.EFFECTOR_PEAK_VOLTAGE)));
            
        effectorRightFX.getConfigurator().apply(new VoltageConfigs()
            .withPeakForwardVoltage(Volts.of(effector.EFFECTOR_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(-effector.EFFECTOR_PEAK_VOLTAGE)));
        
        //Motion Magic Configurations for effector motors
        effectorLeftFX.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(30 * Constants.MASTER_SPEED_MULTIPLIER))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100 * Constants.MASTER_SPEED_MULTIPLIER)));

        effectorRightFX.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(30 * Constants.MASTER_SPEED_MULTIPLIER))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100 * Constants.MASTER_SPEED_MULTIPLIER)));
    
        //Absolute Encoder Configuration for algae motor
        AbsoluteEncoderConfig algaeEncoder = algaeConfig.absoluteEncoder;
        algaeEncoder.positionConversionFactor(1.0);
        algaeEncoder.velocityConversionFactor(1.0);
        algaeEncoder.zeroOffset(0.0);

        //Apply configuration to algae motor
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
        //symmetric start for effector wheels
        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(velocity * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(velocity * Constants.MASTER_SPEED_MULTIPLIER));

    }

    public void start(double velocityLeft, double velocityRight) {
        //asymmetric start for effector wheels
        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(velocityLeft * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(velocityRight * Constants.MASTER_SPEED_MULTIPLIER));

    }

    public void startLock() {
        // Locking coral by rotating effector LOCK_ROTATIONS
        double leftCurrentPos = effectorLeftFX.getPosition().getValueAsDouble();
        double rightCurrentPos = effectorRightFX.getPosition().getValueAsDouble();

        double leftTarget = leftCurrentPos + effector.LOCK_ROTATIONS;
        double rightTarget = rightCurrentPos - effector.LOCK_ROTATIONS; 

        effectorLeftFX.setControl(m_positionVoltage.withPosition(leftTarget));
        effectorRightFX.setControl(m_positionVoltage.withPosition(rightTarget));
    }

    public boolean isCoralDetected() {
        return intakeSensor.getMeasurement().distance_mm < 10;
    }

    public void algaeEffectorUp(double percent) {
        algaeMotor.set(-percent);
    }

    public void algaeEffectorDown(double percent) {
        algaeMotor.set(percent);

    }
}
