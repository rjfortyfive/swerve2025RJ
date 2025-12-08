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

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorLeftFX = new TalonFX(CanIDs.ELEVATOR_LEFT_FX_ID);
    private final TalonFX elevatorRightFX = new TalonFX(CanIDs.ELEVATOR_RIGHT_FX_ID);
    private final DigitalInput bottomlimitSwitch = new DigitalInput(0);
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

    public void checkBottomLimitAndZero(){
        boolean bottomPressed = !bottomlimitSwitch.get();

        if (bottomPressed && !hasZeroed) {
            System.out.println("BOTTOM LIMIT SWITCH PRESSED > ZEROING ELEVATOR");

            elevatorLeftFX.setPosition(0);
            elevatorRightFX.setPosition(0);

            hasZeroed = true;
        }

        if(!bottomPressed) {
            hasZeroed = false;
        }
    }

    public void toPosition(double rotations) {
        System.out.println("Going to " + rotations);
        elevatorLeftFX.setControl(new MotionMagicVoltage(rotations));
    }


    public double getPosition() {
        return elevatorLeftFX.getPosition().getValueAsDouble();
    }

}
