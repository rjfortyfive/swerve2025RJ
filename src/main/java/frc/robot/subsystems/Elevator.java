package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Elevator subsystem controls the vertical movement of the scoring mechanism.
 * 
 * The elevator consists of:
 * - Two TalonFX motors (left master, right follower) that move the elevator up/down
 * - A digital limit switch at the bottom for zeroing/calibration
 * - MotionMagic control for smooth, precise positioning to scoring heights
 * 
 * The elevator moves between several predefined heights:
 * - L1 (0.05 rotations): Bottom position
 * - L2 (15.5 rotations): Second scoring level
 * - L3 (37.5 rotations): Third scoring level
 * - L4 (72.5-73 rotations): Top scoring level (varies by alliance)
 * - INTAKE (1.2 rotations): Position for picking up coral from ground
 */
public class Elevator extends SubsystemBase {
    /** Left elevator motor (master) */
    private final TalonFX elevatorLeftFX = new TalonFX(CanIDs.ELEVATOR_LEFT_FX_ID);
    /** Right elevator motor (follower, inverted) */
    private final TalonFX elevatorRightFX = new TalonFX(CanIDs.ELEVATOR_RIGHT_FX_ID);
    /** Bottom limit switch for zeroing/calibration */
    private final DigitalInput bottomlimitSwitch = new DigitalInput(0);
    /** Tracks whether elevator has been zeroed this cycle */
    private boolean hasZeroed = false;

    public Elevator() {
        //setting Elevator motors to Brake mode
        elevatorLeftFX.setNeutralMode(NeutralModeValue.Brake);
        elevatorRightFX.setNeutralMode(NeutralModeValue.Brake);
        
        //Setting Right elevator motor to invert
        elevatorRightFX.setControl(new Follower(elevatorLeftFX.getDeviceID(), true));

        //Current Limits Configuration for elevator motors
        elevatorLeftFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(elevator.ELEVATOR_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(elevator.ELEVATOR_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));

        elevatorRightFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(elevator.ELEVATOR_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(elevator.ELEVATOR_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));            

        //PID Configuration for elevator motors
        elevatorLeftFX.getConfigurator().apply( new Slot0Configs()
            .withKP(elevator.P_ELEVATOR)
            .withKI(elevator.I_ELEVATOR)
            .withKD(elevator.D_ELEVATOR)
            .withKG(elevator.G_ELEVATOR)
            .withKS(elevator.S_ELEVATOR)
            .withKV(elevator.V_ELEVATOR)
            .withKA(elevator.A_ELEVATOR));

        elevatorRightFX.getConfigurator().apply( new Slot0Configs()
            .withKP(elevator.P_ELEVATOR)
            .withKI(elevator.I_ELEVATOR)
            .withKD(elevator.D_ELEVATOR)
            .withKG(elevator.G_ELEVATOR)
            .withKS(elevator.S_ELEVATOR)
            .withKV(elevator.V_ELEVATOR)
            .withKA(elevator.A_ELEVATOR));        
        
        //Motion Magic Configurations for elevator motors
        elevatorLeftFX.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(elevator.ELEVATOR_CRUISE_VELOCITY * Constants.MASTER_SPEED_MULTIPLIER))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(elevator.ELEVATOR_ACCEL)));

        elevatorRightFX.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(elevator.ELEVATOR_CRUISE_VELOCITY * Constants.MASTER_SPEED_MULTIPLIER))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(elevator.ELEVATOR_ACCEL)));
        
        //Soft Limits Configuration for elevator motors
        elevatorLeftFX.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(elevator.ELEVATOR_UPPER_LIMIT)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(elevator.ELEVATOR_LOWER_LIMIT));

        elevatorRightFX.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(elevator.ELEVATOR_UPPER_LIMIT)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(elevator.ELEVATOR_LOWER_LIMIT));

        
    }
    
    @Override
    public void periodic() {
        checkBottomLimitAndZero();
    }

    /**
     * Checks the bottom limit switch and zeros the elevator encoder when pressed.
     * 
     * This method is called periodically to detect when the elevator reaches the bottom.
     * When the limit switch is pressed, it resets both motor encoders to 0 rotations,
     * providing an absolute reference point for position control.
     * 
     * The hasZeroed flag prevents multiple zero operations in the same switch press cycle.
     */
    public void checkBottomLimitAndZero(){
        // Limit switch is normally-open, so pressed = false (inverted)
        boolean bottomPressed = !bottomlimitSwitch.get();

        if (bottomPressed && !hasZeroed) {
            System.out.println("BOTTOM LIMIT SWITCH PRESSED > ZEROING ELEVATOR");

            // Reset encoder positions to zero
            elevatorLeftFX.setPosition(0);
            elevatorRightFX.setPosition(0);

            // Mark as zeroed to prevent multiple zero operations
            hasZeroed = true;
        }

        // Reset flag when switch is released to allow zeroing again next time
        if(!bottomPressed) {
            hasZeroed = false;
        }
    }

    /**
     * Moves the elevator to a target position using MotionMagic.
     * 
     * MotionMagic provides smooth acceleration/deceleration profiles and precise
     * positioning. The right motor follows the left motor automatically.
     * 
     * @param rotations Target position in rotations (0 = bottom, higher = higher up)
     */
    public void toPosition(double rotations) {
        System.out.println("Going to " + rotations);
        // MotionMagic provides smooth motion profile to target position
        elevatorLeftFX.setControl(new MotionMagicVoltage(rotations));
        // Right motor follows automatically via Follower configuration
    }

    /**
     * Gets the current elevator position.
     * 
     * @return Current position in rotations (0 = bottom reference)
     */
    public double getPosition() {
        return elevatorLeftFX.getPosition().getValueAsDouble();
    }

}
