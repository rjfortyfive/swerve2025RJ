package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.state.RobotState;
import frc.robot.state.RobotStateManager;

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

    private final RobotStateManager stateManager;

    // Target setpoints (add these to your Constants.elevator)
    // public static final double INTAKE_CORAL_POS = ...;
    // public static final double SCORE_CORAL_POS  = ...;
    // public static final double INTAKE_ALGAE_POS = ...;
    // public static final double SCORE_ALGAE_POS  = ...;
    // public static final double CLIMB_POS        = ...;

    public Elevator(RobotStateManager stateManager) {
        this.stateManager = stateManager;

        System.out.println("Creating new motor with ID " + Constants.elevator.ELEVATOR_LEFT_FX_ID);
        elevatorLeftFX.setNeutralMode(NeutralModeValue.Brake);
        elevatorRightFX.setControl(new Follower(elevatorLeftFX.getDeviceID(), true));

        // all your existing configurator code here…
        // (no change to configs)
    }

    @Override
    public void periodic() {
        checkBottomLimitAndZero();

        switch (stateManager.getState()) {
            case INTAKE_CORAL:
                toPosition(Constants.elevator.INTAKE_CORAL_POS);
                break;
            case INTAKE_ALGAE:
                toPosition(Constants.elevator.INTAKE_ALGAE_POS);
                break;
            case SCORE_CORAL:
                toPosition(Constants.elevator.SCORE_CORAL_POS);
                break;
            case SCORE_ALGAE:
                toPosition(Constants.elevator.SCORE_ALGAE_POS);
                break;
            case CLIMB:
                toPosition(Constants.elevator.CLIMB_POS);
                break;
            case IDLE:
            case DRIVE:
            case ALIGN_VISION:
            case MANUAL_OVERRIDE:
            case DISABLED:
            default:
                // Hold current position (or you can just do nothing)
                break;
        }
    }

    public void checkBottomLimitAndZero() {
        boolean bottomPressed = !bottomlimitSwitch.get();

        if (bottomPressed && !hasZeroed) {
            System.out.println("BOTTOM LIMIT SWITCH PRESSED > ZEROING ELEVATOR");
            elevatorLeftFX.setPosition(0);
            elevatorRightFX.setPosition(0);
            hasZeroed = true;
        }

        if (!bottomPressed) {
            hasZeroed = false;
        }
    }

    public void toPosition(double rotations) {
        elevatorLeftFX.setControl(new MotionMagicVoltage(rotations));
    }

    public double getPosition() {
        return elevatorLeftFX.getPosition().getValueAsDouble();
    }
}
