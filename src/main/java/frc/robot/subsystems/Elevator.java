package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator extends SubsystemBase {
    public static TalonFX elevatorLeftFX = new TalonFX(Constants.elevator.ELEVATOR_LEFT_FX_ID);
    public static TalonFX elevatorRightFX = new TalonFX(Constants.elevator.ELEVATOR_RIGHT_FX_ID);
    public static DigitalInput bottomlimitSwitch = new DigitalInput(0);
//private final static MotionMagicVoltage motionControl = new MotionMagicVoltage(0);

    public Elevator() {
        System.out.println("Creating new motor with ID " + Constants.elevator.ELEVATOR_LEFT_FX_ID);
        elevatorLeftFX.setNeutralMode(NeutralModeValue.Brake);
        elevatorRightFX.setControl(new Follower(elevatorLeftFX.getDeviceID(), true));

        elevatorLeftFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.elevator.ELEVATOR_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.elevator.ELEVATOR_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));

        elevatorRightFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.elevator.ELEVATOR_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.elevator.ELEVATOR_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));            

        elevatorLeftFX.getConfigurator().apply( new Slot0Configs()
            .withKP(Constants.elevator.P_ELEVATOR)
            .withKI(Constants.elevator.I_ELEVATOR)
            .withKD(Constants.elevator.D_ELEVATOR)
            .withKG(Constants.elevator.G_ELEVATOR)
            .withKS(Constants.elevator.S_ELEVATOR)
            .withKV(Constants.elevator.V_ELEVATOR)
            .withKA(Constants.elevator.A_ELEVATOR));

        elevatorRightFX.getConfigurator().apply( new Slot0Configs()
            .withKP(Constants.elevator.P_ELEVATOR)
            .withKI(Constants.elevator.I_ELEVATOR)
            .withKD(Constants.elevator.D_ELEVATOR)
            .withKG(Constants.elevator.G_ELEVATOR)
            .withKS(Constants.elevator.S_ELEVATOR)
            .withKV(Constants.elevator.V_ELEVATOR)
            .withKA(Constants.elevator.A_ELEVATOR));        
            
        elevatorLeftFX.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(Constants.elevator.ELEVATOR_CRUISE_VELOCITY * Constants.MASTER_SPEED_MULTIPLIER))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(Constants.elevator.ELEVATOR_ACCEL)));

        elevatorRightFX.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(Constants.elevator.ELEVATOR_CRUISE_VELOCITY * Constants.MASTER_SPEED_MULTIPLIER))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(Constants.elevator.ELEVATOR_ACCEL)));
        
        elevatorLeftFX.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Constants.elevator.ELEVATOR_UPPER_LIMIT)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(Constants.elevator.ELEVATOR_LOWER_LIMIT));

        elevatorRightFX.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Constants.elevator.ELEVATOR_UPPER_LIMIT)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(Constants.elevator.ELEVATOR_LOWER_LIMIT));

        
    }

    public static void toPosition(double rotations) {
        System.out.println("Going to " + rotations);
        //elevatorLeftFX.setControl(motionControl.withPosition(rotations));
        elevatorLeftFX.setControl(new MotionMagicVoltage(rotations));
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
    }


    public static void manualOffset(boolean direction) {
        var currentPos = elevatorLeftFX.getPosition().getValueAsDouble();
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

        elevatorLeftFX.setControl(new MotionMagicVoltage(currentPos));
        System.out.println("New Position: " + currentPos);
        SmartDashboard.putNumber("Offset", currentPos);
        return;
    }


    public static double getPosition() {
        return elevatorLeftFX.getPosition().getValueAsDouble();
    }
}
