package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.state.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator extends SubsystemBase {
    private final RobotStateManager sm;

    public final static TalonFX elevatorLeftFX = new TalonFX(Constants.elevator.ELEVATOR_LEFT_FX_ID);
    public final static TalonFX elevatorRightFX = new TalonFX(Constants.elevator.ELEVATOR_RIGHT_FX_ID);
    public final static DigitalInput bottomlimitSwitch = new DigitalInput(0);
    private boolean hasZeroed = false;

    public Elevator(RobotStateManager sm) {
        this.sm = sm;

        // 🔒 ALL YOUR CONFIGURATION IS UNTOUCHED
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

        switch (sm.getState()) {

            case INTAKE_CORAL:
                toPosition(Constants.elevator.level.INTAKE);
                break;

            case SCORE_CORAL:
                toPosition(Constants.elevator.level.L3);
                break;

            case INTAKE_ALGAE:
                toPosition(Constants.elevator.level.ALGAE);
                break;

            case SCORE_ALGAE:
                toPosition(Constants.elevator.level.ALGAE_HIGH);
                break;

            case CLIMB:
                toPosition(Constants.elevator.level.CLIMB);
                break;

            default:
                // Hold last commanded position
                break;
        }
    }

    private void checkBottomLimitAndZero(){
        boolean bottomPressed = !bottomlimitSwitch.get();

        if (bottomPressed && !hasZeroed) {
            elevatorLeftFX.setPosition(0);
            elevatorRightFX.setPosition(0);
            hasZeroed = true;
        }

        if (!bottomPressed)
            hasZeroed = false;
    }

    public void toPosition(double rotations) {
        elevatorLeftFX.setControl(new MotionMagicVoltage(rotations));
    }

    public double getPosition() {
        return elevatorLeftFX.getPosition().getValueAsDouble();
    }
}
