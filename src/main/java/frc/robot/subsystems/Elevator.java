package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator extends SubsystemBase {
    public static TalonFX liftLeft = new TalonFX(Constants.elevator.LIFT_LEFT_ID);
    public static TalonFX liftRight = new TalonFX(Constants.elevator.LIFT_RIGHT_ID);
    public static DigitalInput bottomlimitSwitch = new DigitalInput(0);
    public static DigitalInput toplimitSwitch = new DigitalInput(1);
    public final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(1);
    private final static MotionMagicVoltage motionControl = new MotionMagicVoltage(0).withSlot(0);

    public Elevator() {
        System.out.println("Creating new motor with ID " + Constants.elevator.LIFT_LEFT_ID);
        liftLeft.setNeutralMode(NeutralModeValue.Brake);
        liftRight.setControl(new Follower(liftLeft.getDeviceID(), true));

        configureElevator();
    }

    public static void toPosition(double rotations) {
        System.out.println("Going to " + rotations);
        if (toplimitSwitch.get()) {
            // System.out.println(motionControl.withPosition(rotations));
            liftLeft.setControl(motionControl.withPosition(rotations));
            if (rotations == Constants.elevator.level.L1) {
                Constants.elevator.level.activeLevel = 1;
            } else if (rotations == Constants.elevator.level.L2) {
                Constants.elevator.level.activeLevel = 2;
            } else if (rotations == Constants.elevator.level.L3) {
                Constants.elevator.level.activeLevel = 3;
            } else if (rotations == Constants.elevator.level.L4) {
                Constants.elevator.level.activeLevel = 4;
            } else if (rotations == Constants.elevator.level.intake) {
                Constants.elevator.level.activeLevel = 5;
            } else {
                Constants.elevator.level.activeLevel = 0;
            }
            ;
            // var status = liftLeft.setControl(m_velocityVoltage.withVelocity(-10));

            // liftLeft.set(5);
            // System.out.println(liftLeft.getPosition() + " rotations reached.");
        } else {
            liftLeft.set(0);
            liftLeft.setControl(m_velocityVoltage.withVelocity(0));
        }
        return;
    }

    public static void toBottom() {
        while (bottomlimitSwitch.get()) {
            // liftLeft.setControl(motionControl.withPosition(0));
            liftLeft.setControl(m_velocityVoltage.withVelocity(10 * Constants.MASTER_SPEED_MULTIPLIER));
        }
        liftLeft.setControl(m_velocityVoltage.withVelocity(0));
    }

    public static void manualControl(double velocity) {
        double desiredRotationsPerSecond;
        double liftPosition = liftLeft.getPosition().getValueAsDouble();

        if ((liftPosition < 0.2) && (velocity < 0)) {
            desiredRotationsPerSecond = 0;
        } else if ((liftPosition > 60) && (velocity < 0)) {
            desiredRotationsPerSecond = velocity * -(75 - liftPosition) * 1;
        } else if ((liftPosition < 9) && (velocity > 0)) {
            desiredRotationsPerSecond = velocity * -liftPosition * 1;
        } else {
            desiredRotationsPerSecond = velocity * -10;
        }
        liftLeft.setControl(
                m_velocityVoltage.withVelocity(desiredRotationsPerSecond * Constants.MASTER_SPEED_MULTIPLIER));
    }

    public static void manualOffset(boolean direction) {
        var currentPos = liftLeft.getPosition().getValueAsDouble();
        if (direction == false) {
            if (Constants.elevator.level.activeLevel == 1) {
                Constants.elevator.level.L1 = Constants.elevator.level.L1 - 0.3;
                currentPos = Constants.elevator.level.L1;

            } else if (Constants.elevator.level.activeLevel == 2) {
                Constants.elevator.level.L2 = Constants.elevator.level.L2 - 0.3;
                currentPos = Constants.elevator.level.L2;

            } else if (Constants.elevator.level.activeLevel == 3) {
                Constants.elevator.level.L3 = Constants.elevator.level.L3 - 0.3;
                currentPos = Constants.elevator.level.L3;

            } else if (Constants.elevator.level.activeLevel == 4) {
                Constants.elevator.level.L4 = Constants.elevator.level.L4 - 0.3;
                currentPos = Constants.elevator.level.L4;

            } else if (Constants.elevator.level.activeLevel == 5) {
                Constants.elevator.level.intake = Constants.elevator.level.intake - 0.3;
                currentPos = Constants.elevator.level.intake;
            }

        } else {
            if (Constants.elevator.level.activeLevel == 1) {
                Constants.elevator.level.L1 = Constants.elevator.level.L1 + 0.3;
                currentPos = Constants.elevator.level.L1;

            } else if (Constants.elevator.level.activeLevel == 2) {
                Constants.elevator.level.L2 = Constants.elevator.level.L2 + 0.3;
                currentPos = Constants.elevator.level.L2;

            } else if (Constants.elevator.level.activeLevel == 3) {
                Constants.elevator.level.L3 = Constants.elevator.level.L3 + 0.3;
                currentPos = Constants.elevator.level.L3;

            } else if (Constants.elevator.level.activeLevel == 4) {
                Constants.elevator.level.L4 = Constants.elevator.level.L4 + 0.3;
                currentPos = Constants.elevator.level.L4;

            } else if (Constants.elevator.level.activeLevel == 5) {
                Constants.elevator.level.intake = Constants.elevator.level.intake + 0.3;
                currentPos = Constants.elevator.level.intake;
            }

        }

        liftLeft.setControl(motionControl.withPosition(currentPos));
        System.out.println("New Position: " + currentPos);
        SmartDashboard.putNumber("Offset", currentPos);
        return;
    }

    public static void configureElevator() {
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        var limitConfigs = elevatorConfig.CurrentLimits;

        limitConfigs.SupplyCurrentLimit = 60;
        limitConfigs.StatorCurrentLimit = 120;
        limitConfigs.StatorCurrentLimitEnable = true;

        elevatorConfig.Voltage.withPeakForwardVoltage(Volts.of(12 * Constants.masterVoltageMultiplier))
                .withPeakReverseVoltage(Volts.of(-12 * Constants.masterVoltageMultiplier)); // 12 and -12

        MotionMagicConfigs motionConfig = elevatorConfig.MotionMagic;
        motionConfig.withMotionMagicCruiseVelocity(RotationsPerSecond.of(90 * Constants.MASTER_SPEED_MULTIPLIER))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(500)); // Should be 90 and 500
        // PID CONSTANTS
        elevatorConfig.Slot0.kP = 1.5; // 3
        elevatorConfig.Slot0.kI = 0.00; // 0.03
        elevatorConfig.Slot0.kD = 0.01; // 0.01
        elevatorConfig.Slot0.kV = 0.13; // 0.13
        elevatorConfig.Slot0.kG = 0.42; // 0.22
        elevatorConfig.Slot0.kS = 0;
        elevatorConfig.Slot0.kA = 0;

        elevatorConfig.Slot1.kS = 0.3; // To account for friction, add 0.1 V of static feedforward
        elevatorConfig.Slot1.kG = 0.2; // Gravity constant, determined by gear ratio
        elevatorConfig.Slot1.kV = 0.13; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                                        // volts / rotation per second
        elevatorConfig.Slot1.kP = 0.12; // An error of 1 rotation per second results in 0.11 V output
        elevatorConfig.Slot1.kI = 0.06; // No output for integrated error
        elevatorConfig.Slot1.kD = 0.001; // No output for error derivative

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = liftLeft.getConfigurator().apply(elevatorConfig);
            System.out.println("Elevator configs applied successfully");
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        System.out.println("Left elevator: " + liftLeft.getDeviceID());
        System.out.println("Right elevator: " + liftRight.getDeviceID());

        liftLeft.setPosition(0);

    }

    public static double getPosition() {
        return liftLeft.getPosition().getValueAsDouble();
    }
}
