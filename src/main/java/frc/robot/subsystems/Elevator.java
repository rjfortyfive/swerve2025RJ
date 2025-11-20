package frc.robot.subsystems;

import frc.robot.Constants;

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
    public final static TalonFX elevatorLeftFX = new TalonFX(Constants.elevator.ELEVATOR_LEFT_FX_ID);
    public final static TalonFX elevatorRightFX = new TalonFX(Constants.elevator.ELEVATOR_RIGHT_FX_ID);
    public final static DigitalInput bottomlimitSwitch = new DigitalInput(0);
    private boolean hasZeroed = false;
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
        //elevatorLeftFX.setControl(motionControl.withPosition(rotations));
        elevatorLeftFX.setControl(new MotionMagicVoltage(rotations));
    }


    public double getPosition() {
        return elevatorLeftFX.getPosition().getValueAsDouble();
    }

}
