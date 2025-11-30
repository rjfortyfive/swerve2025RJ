package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.state.RobotState;
import frc.robot.state.RobotStateManager;

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

    private static TalonFX effectorLeftFX = new TalonFX(Constants.effector.EFFECTOR_LEFT_FX_ID);
    private static TalonFX effectorRightFX = new TalonFX(Constants.effector.EFFECTOR_RIGHT_FX_ID);

    private static SparkMax algaeMotor = new SparkMax(1, MotorType.kBrushed);

    private final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);
    private final static PositionVoltage m_positionVoltage = new PositionVoltage(0);

    private final RobotStateManager stateManager;

    public Effector(RobotStateManager stateManager) {
        this.stateManager = stateManager;

        // your existing CTRE + REV config code here...
    }

    @Override
    public void periodic() {
        RobotState s = stateManager.getState();

        switch (s) {
            case INTAKE_CORAL:
                // Intake inward until LaserCan sees a coral, then auto-idle
                start(Constants.effector.CORAL_INTAKE_VEL);
                algaeMotor.set(0); // algae off

                if (isCoralDetected() && stateManager.getTimeInState() > 0.2) {
                    stop();
                    stateManager.setState(RobotState.IDLE);
                }
                break;

            case SCORE_CORAL:
                // Spit coral outwards for a bit, then auto-idle
                start(Constants.effector.CORAL_SCORE_VEL);
                algaeMotor.set(0);

                if (stateManager.getTimeInState() > 0.5) {
                    stop();
                    stateManager.setState(RobotState.IDLE);
                }
                break;

            case INTAKE_ALGAE:
                // Use algae motor to pull in
                start(0);
                algaeEffectorDown(Constants.effector.ALGAE_IN_PERCENT);

                // Auto-transition purely time based (unless you later add a sensor)
                if (stateManager.getTimeInState() > 1.0) {
                    algaeMotor.set(0);
                    stateManager.setState(RobotState.IDLE);
                }
                break;

            case SCORE_ALGAE:
                // push algae out
                start(0);
                algaeEffectorUp(Constants.effector.ALGAE_OUT_PERCENT);

                if (stateManager.getTimeInState() > 0.7) {
                    algaeMotor.set(0);
                    stateManager.setState(RobotState.IDLE);
                }
                break;

            default:
                // Any other state → stop everything
                stop();
                algaeMotor.set(0);
                break;
        }
    }

    public void moveToPositions(double leftPos, double rightPos) {
        effectorLeftFX.setControl(m_positionVoltage.withPosition(leftPos));
        effectorRightFX.setControl(m_positionVoltage.withPosition(rightPos));
    }

    public void stop() {
        effectorLeftFX.set(0);
        effectorRightFX.set(0);
    }

    public void start(double velocity) {
        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(velocity * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(-velocity * Constants.MASTER_SPEED_MULTIPLIER));
    }

    public void start(double velocityLeft, double velocityRight) {
        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(velocityLeft * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(velocityRight * Constants.MASTER_SPEED_MULTIPLIER));
    }

    public void startLock() {
        double leftCurrentPos = effectorLeftFX.getPosition().getValueAsDouble();
        double rightCurrentPos = effectorRightFX.getPosition().getValueAsDouble();

        double leftTarget = leftCurrentPos + Constants.intake.LOCK_ROTATIONS;
        double rightTarget = rightCurrentPos - Constants.intake.LOCK_ROTATIONS;

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
