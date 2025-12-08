package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.effector;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.Utils;
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
    private final LaserCan effectorSensor = new LaserCan(CanIDs.EFFECTOR_LASER_ID);

    private final TalonFX effectorLeftFX = new TalonFX(CanIDs.EFFECTOR_LEFT_FX_ID);
    private final TalonFX effectorRightFX = new TalonFX(CanIDs.EFFECTOR_RIGHT_FX_ID);

    private final SparkMax algaeMotor = new SparkMax(1, MotorType.kBrushed);

    private final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);

    private final static PositionVoltage m_positionVoltage = new PositionVoltage(0);
    
    // Simulation support for coral detection
    private boolean simCoralPresent = false; // Start with no coral in sim (for intake)
    private double simActionStartTime = 0.0;
    private double simReverseStartTime = 0.0; // Track when reverse phase starts
    private boolean simIsIntaking = false; // Track if we're intaking or outtaking
    private boolean simInReversePhase = false; // Track if we're in reverse phase of intake
    private double lastIntakeForwardVelocity = 0.0; // Track last intake forward velocity to detect changes
    private static final double SIM_OUTTAKE_DURATION = 0.5; // Time in seconds for coral to be ejected in sim
    private static final double SIM_INTAKE_DURATION = 0.3; // Time in seconds for coral to appear during intake in sim
    private static final double SIM_REVERSE_DURATION = 0.4; // Time in seconds for reverse phase to complete

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
        
        // Reset simulation state when stopping
        if (Utils.isSimulation()) {
            simActionStartTime = 0.0;
            simReverseStartTime = 0.0;
            simInReversePhase = false;
            lastIntakeForwardVelocity = 0.0;
            // Don't reset coral state here - let it persist for next action
        }
    }

    public void start(double velocity) {
        //symmetric start for effector wheels
        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(velocity * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(velocity * Constants.MASTER_SPEED_MULTIPLIER));
        
        // Track when action starts in simulation
        if (Utils.isSimulation()) {
            if (velocity > 0) {
                // Positive velocity could be intake forward (30, 20) or outtake (40)
                // For intake: start with no coral, coral appears after delay
                // For outtake: start with coral, coral disappears after delay
                // We'll use velocity to distinguish: 30-20 = intake, 40 = outtake
                if (velocity >= 35) {
                    // High velocity (40) = outtake
                    simIsIntaking = false;
                    simInReversePhase = false;
                    simCoralPresent = true; // Coral present for outtake
                    simActionStartTime = Timer.getFPGATimestamp();
                    simReverseStartTime = 0.0;
                    lastIntakeForwardVelocity = 0.0; // Reset
                } else {
                    // Lower velocity (20-30) = intake forward
                    // Only reset timer if velocity changed or if we're starting a new intake phase
                    // This prevents timer from resetting on repeated calls with same velocity
                    boolean velocityChanged = Math.abs(velocity - lastIntakeForwardVelocity) > 1.0;
                    if (!simInReversePhase && (velocityChanged || simActionStartTime == 0.0)) {
                        // Reset timer only if velocity changed or timer hasn't been started
                        simActionStartTime = Timer.getFPGATimestamp();
                    }
                    lastIntakeForwardVelocity = velocity;
                    simIsIntaking = true;
                    simInReversePhase = false;
                    simReverseStartTime = 0.0;
                    // Coral state will be managed by isCoralDetected based on elapsed time
                }
            } else {
                // Negative velocity = reverse phase of intake
                simIsIntaking = true;
                simInReversePhase = true;
                simReverseStartTime = Timer.getFPGATimestamp();
                simCoralPresent = false; // Coral disappears during reverse
                lastIntakeForwardVelocity = 0.0; // Reset
            }
        }
    }

    public void start(double velocityLeft, double velocityRight) {
        //asymmetric start for effector wheels
        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(velocityLeft * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(velocityRight * Constants.MASTER_SPEED_MULTIPLIER));
        
        // Track when action starts in simulation (asymmetric is typically outtake)
        if (Utils.isSimulation()) {
            simActionStartTime = Timer.getFPGATimestamp();
            simIsIntaking = false;
            simCoralPresent = true; // Coral present for outtake
        }
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
        // In simulation, simulate coral detection behavior
        if (Utils.isSimulation()) {
            if (simIsIntaking) {
                if (simInReversePhase && simReverseStartTime > 0) {
                    // We're in reverse phase - coral disappears then reappears
                    double reverseElapsed = Timer.getFPGATimestamp() - simReverseStartTime;
                    if (reverseElapsed < 0.2) {
                        // Brief disappearance during reverse
                        simCoralPresent = false;
                    } else if (reverseElapsed >= SIM_REVERSE_DURATION) {
                        // After reverse duration, coral is locked in place
                        simCoralPresent = true;
                    }
                } else if (simActionStartTime > 0) {
                    // Forward intake phase - coral appears after delay
                    double elapsed = Timer.getFPGATimestamp() - simActionStartTime;
                    if (elapsed >= SIM_INTAKE_DURATION) {
                        simCoralPresent = true; // Coral picked up
                    } else {
                        simCoralPresent = false; // Still picking up
                    }
                }
            } else {
                // Outtake phase
                if (simActionStartTime > 0) {
                    double elapsed = Timer.getFPGATimestamp() - simActionStartTime;
                    if (elapsed >= SIM_OUTTAKE_DURATION) {
                        simCoralPresent = false; // Coral ejected
                    }
                }
            }
            return simCoralPresent;
        }
        
        // On real robot, use actual LaserCan sensor
        try {
            return effectorSensor.getMeasurement().distance_mm < 10;
        } catch (Exception e) {
            // If sensor fails, assume no coral detected to prevent blocking
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
