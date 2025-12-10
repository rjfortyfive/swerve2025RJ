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


/**
 * Effector subsystem manages the coral scoring mechanism and algae removal.
 * 
 * The effector consists of:
 * - Two TalonFX motors (left and right) that control scoring wheels
 * - A LaserCan sensor that detects when coral is present
 * - A SparkMax brushed motor for algae removal mechanism
 * 
 * The scoring wheels can run symmetrically (same speed both directions) or
 * asymmetrically (different speeds) for special scoring techniques at L1.
 * The right motor is inverted to allow opposing rotation for locking coral.
 */
public class Effector extends SubsystemBase {
    /** LaserCan sensor to detect coral presence (distance < 10mm = coral present) */
    private final LaserCan effectorSensor = new LaserCan(CanIDs.EFFECTOR_LASER_ID);

    /** Left effector motor (TalonFX) */
    private final TalonFX effectorLeftFX = new TalonFX(CanIDs.EFFECTOR_LEFT_FX_ID);
    /** Right effector motor (TalonFX) - inverted for opposing rotation */
    private final TalonFX effectorRightFX = new TalonFX(CanIDs.EFFECTOR_RIGHT_FX_ID);

    /** Algae removal motor (SparkMax brushed) */
    private final SparkMax algaeMotor = new SparkMax(1, MotorType.kBrushed);

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
    /**
     * Checks if both effector motors have reached their target positions.
     * Used to determine when lock rotation or scoring rotation is complete.
     * 
     * @param leftTarget Target position for left motor in rotations
     * @param rightTarget Target position for right motor in rotations
     * @return true if both motors are within 0.02 rotations of their targets
     */
    public boolean coralAtPosition(double leftTarget, double rightTarget) {
        double leftErr  = Math.abs(getLeftPosition()  - leftTarget);
        double rightErr = Math.abs(getRightPosition() - rightTarget);
        return leftErr < 0.02 && rightErr < 0.02;  // 0.02 rotations tolerance
    }

    public void stop() {
        effectorLeftFX.set(0);
        effectorRightFX.set(0);

    }

    /**
     * Starts effector wheels at the same velocity (symmetric operation).
     * Used for normal intake/outtake operations.
     * 
     * @param velocity Target velocity in rotations per second (applied to both motors)
     */
    public void start(double velocity) {
        // Symmetric start for effector wheels - both run at same speed
        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(velocity * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(velocity * Constants.MASTER_SPEED_MULTIPLIER));
    }

    /**
     * Starts effector wheels at different velocities (asymmetric operation).
     * Used for L1 scoring where different speeds help place coral accurately.
     * 
     * @param velocityLeft Target velocity for left motor in RPS
     * @param velocityRight Target velocity for right motor in RPS
     */
    public void start(double velocityLeft, double velocityRight) {
        // Asymmetric start for effector wheels - different speeds for each motor
        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(velocityLeft * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(velocityRight * Constants.MASTER_SPEED_MULTIPLIER));
    }

    /**
     * Locks coral in place by rotating the effector wheels in opposite directions.
     * 
     * Uses MotionMagic position control to rotate:
     * - Left motor: forward by LOCK_ROTATIONS
     * - Right motor: backward by LOCK_ROTATIONS (inverted, so this rotates forward relative to mechanism)
     * 
     * This creates a pinching/twisting motion that secures the coral.
     */
    public void startLock() {
        // Get current positions
        double leftCurrentPos = effectorLeftFX.getPosition().getValueAsDouble();
        double rightCurrentPos = effectorRightFX.getPosition().getValueAsDouble();

        // Calculate target positions - rotate in opposite directions
        // Left rotates forward (+), right rotates backward (-) due to inversion
        double leftTarget = leftCurrentPos + effector.LOCK_ROTATIONS;
        double rightTarget = rightCurrentPos - effector.LOCK_ROTATIONS; 

        // Command motors to target positions using MotionMagic
        effectorLeftFX.setControl(m_positionVoltage.withPosition(leftTarget));
        effectorRightFX.setControl(m_positionVoltage.withPosition(rightTarget));
    }

    /**
     * Checks if coral is detected by the LaserCan sensor.
     * 
     * @return true if coral is detected (distance < 10mm), false otherwise
     *         Returns false if sensor fails to prevent blocking commands
     */
    public boolean isCoralDetected() {
        try {
            // Coral is detected when sensor reads distance less than 10mm
            return effectorSensor.getMeasurement().distance_mm < 10;
        } catch (Exception e) {
            // If sensor fails (disconnected, error, etc.), assume no coral detected
            // This prevents commands from blocking waiting for coral that will never be detected
            return false;
        }
    }

    public void moveAlgaeEffector(double percent) {
        algaeMotor.set(percent);
    }

    public void stopAlgaeEffector(){
        algaeMotor.set(0);
    }
}
