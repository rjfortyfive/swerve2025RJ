package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Effector extends SubsystemBase {
    private static LaserCan intakeSensor = new LaserCan(2);

    private static TalonFX effectorLeftFX = new TalonFX(Constants.effector.EFFECTOR_LEFT_FX_ID);
    private static TalonFX effectorRightFX = new TalonFX(Constants.effector.EFFECTOR_RIGHT_FX_ID);

    private static SparkMax algaeMotor = new SparkMax(1, MotorType.kBrushed);

    private static Timer effectorTimer = new Timer();
    private static Timer algaeTimer = new Timer();

    private final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);

    private final static PositionVoltage m_positionVoltage = new PositionVoltage(0);

    private volatile static boolean isAlgaeOut = false;

    private final Elevator m_elevator = new Elevator();

    public Effector() {

        effectorLeftFX.setNeutralMode(NeutralModeValue.Brake);
        effectorRightFX.setNeutralMode(NeutralModeValue.Brake);

        effectorTimer = new Timer();

        effectorLeftFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.effector.EFFECTOR_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.effector.EFFECTOR_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));

        effectorRightFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.effector.EFFECTOR_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.effector.EFFECTOR_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));
        
        effectorLeftFX.getConfigurator().apply( new Slot0Configs()
            .withKP(Constants.effector.P_EFFECTOR)
            .withKI(Constants.effector.I_EFFECTOR)
            .withKD(Constants.effector.D_EFFECTOR)
            .withKG(Constants.effector.G_EFFECTOR)
            .withKS(Constants.effector.S_EFFECTOR)
            .withKV(Constants.effector.V_EFFECTOR)
            .withKA(Constants.effector.A_EFFECTOR));
            
        effectorRightFX.getConfigurator().apply( new Slot0Configs()
            .withKP(Constants.effector.P_EFFECTOR)
            .withKI(Constants.effector.I_EFFECTOR)
            .withKD(Constants.effector.D_EFFECTOR)
            .withKG(Constants.effector.G_EFFECTOR)
            .withKS(Constants.effector.S_EFFECTOR)
            .withKV(Constants.effector.V_EFFECTOR)
            .withKA(Constants.effector.A_EFFECTOR));
        
        effectorLeftFX.getConfigurator().apply(new VoltageConfigs()
            .withPeakForwardVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE)));
            
        effectorRightFX.getConfigurator().apply(new VoltageConfigs()
            .withPeakForwardVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE)));
        
        effectorLeftFX.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(30 * Constants.MASTER_SPEED_MULTIPLIER))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100 * Constants.MASTER_SPEED_MULTIPLIER)));

        effectorRightFX.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(30 * Constants.MASTER_SPEED_MULTIPLIER))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100 * Constants.MASTER_SPEED_MULTIPLIER)));
    }

    @Override
    public void periodic() {

    }

    /**
     * Stops all intake and effector motors immediately.
     */
    public void stopIntake() {
        effectorLeftFX.set(0);
        effectorRightFX.set(0);

    }

    public void startIntake() {
        // Turn on intake and effector wheels
        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(40 * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(-40 * Constants.MASTER_SPEED_MULTIPLIER));

    }

    public void startLock() {
        // Turn on intake
        double leftCurrentPos = effectorLeftFX.getPosition().getValueAsDouble();
        double rightCurrentPos = effectorRightFX.getPosition().getValueAsDouble();

        double leftTarget = leftCurrentPos + Constants.intake.lockRotations;
        double rightTarget = rightCurrentPos - Constants.intake.lockRotations; 

        effectorLeftFX.setControl(m_positionVoltage.withPosition(leftTarget));
        effectorRightFX.setControl(m_positionVoltage.withPosition(rightTarget));
    }

    public void reverseLock() {

        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(-15 * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(15 * Constants.MASTER_SPEED_MULTIPLIER));
    }

    public void startOutTake() {
        // Turn on intake
        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(40 * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(-40 * Constants.MASTER_SPEED_MULTIPLIER));

    }

    public boolean isCoralDetected() {
        return intakeSensor.getMeasurement().distance_mm < 10;
    }

    public boolean isCoralNotDetected() {
        return intakeSensor.getMeasurement().distance_mm > 10;
    }


    public void outtakeUntilDetected() {
        if (m_elevator.getPosition() < 5) {
            asymmetricalOuttake(null, null);
        }
        else {
            while (intakeSensor.getMeasurement().distance_mm < 10) {
            symmetricalOuttake(null);
            }
        }
        effectorLeftFX.set(0);
        effectorRightFX.set(0);
        effectorTimer.stop();
        effectorTimer.reset();


    }

    public static void symmetricalOuttake(Double velocity) {
        double motorSpeed;
        if (velocity != null) {
            motorSpeed = velocity;
        }
        else {
            motorSpeed = Constants.effector.defaultVelocity;
        }
        effectorTimer.start();
        while (effectorTimer.get() < 2) {
            effectorLeftFX.setControl(m_velocityVoltage.withVelocity(motorSpeed * Constants.MASTER_SPEED_MULTIPLIER));
            effectorRightFX.setControl(m_velocityVoltage.withVelocity(-motorSpeed * Constants.MASTER_SPEED_MULTIPLIER));
        }
        effectorLeftFX.set(0);
        effectorRightFX.set(0);
        effectorTimer.stop();
        effectorTimer.reset();

    }

    public static void asymmetricalOuttake(Double velocityLeft, Double velocityRight) {
        System.out.println(effectorLeftFX.getPosition().getValueAsDouble());
        double motorSpeedL;
        double motorSpeedR;
        if (velocityLeft != null) {
            motorSpeedL = velocityLeft;
        }
        else {
            motorSpeedL = 30 * Constants.MASTER_SPEED_MULTIPLIER; // 30
        }
        if (velocityRight != null) {
            motorSpeedR = velocityRight;
        }
        else {
            motorSpeedR = 10 * Constants.MASTER_SPEED_MULTIPLIER; // 10
        }
        effectorTimer.start();
        while (effectorTimer.get() < 1.5) {
            effectorLeftFX.setControl(m_velocityVoltage.withVelocity(motorSpeedL));
            effectorRightFX.setControl(m_velocityVoltage.withVelocity(-motorSpeedR));
        }

        effectorLeftFX.set(0);
        effectorRightFX.set(0);
        effectorTimer.stop();
        effectorTimer.reset();
    }

    public static void manualControl(double velocityLeft, Double velocityRight) {
        if (velocityRight == null) {
            velocityRight = -velocityLeft;
        }
        effectorLeftFX.setControl(m_velocityVoltage.withVelocity(velocityLeft * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRightFX.setControl(m_velocityVoltage.withVelocity(velocityRight * Constants.MASTER_SPEED_MULTIPLIER));

    }

    public static void algaeEffectorUp(Double time) {
        algaeTimer.start();

        if (time == null) {
            time = 0.1; //0.10
        }
        else {
        }

        while (algaeTimer.get() < time) {
            algaeMotor.set(-80); //-80
        }
        algaeMotor.set(0);
        isAlgaeOut = true;

        algaeTimer.stop();
        algaeTimer.reset();

    }

    public static void algaeEffectorDown() {
        algaeTimer.start();

        while (algaeTimer.get() < 0.1) { //0.08
            algaeMotor.set(80); //80
        }
        algaeMotor.set(0);
        isAlgaeOut = false;

        algaeTimer.stop();
        algaeTimer.reset();

    }

    public static void toggleAlgae() {
        if (isAlgaeOut) {
            algaeEffectorDown();
        }
        else {
            algaeEffectorUp(null);
        }
    }
}
