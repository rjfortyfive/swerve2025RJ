package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
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

    private static TalonFX effectorLeft = new TalonFX(Constants.effector.EFFECTOR_LEFT_ID);
    private static TalonFX effectorRight = new TalonFX(Constants.effector.EFFECTOR_RIGHT_ID);

    private static TalonFX intakeLeft = new TalonFX(Constants.intake.INTAKE_LEFT_ID);
    private static TalonFX intakeRight = new TalonFX(Constants.intake.INTAKE_RIGHT_ID);

    private static SparkMax algaeMotor = new SparkMax(1, MotorType.kBrushed);

    private static Timer effectorTimer = new Timer();
    private static Timer algaeTimer = new Timer();

    private final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

    private final static PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

    private volatile static boolean isAlgaeOut = false;

    // for speed-controlled bump rotations
    private double m_initialLeftRot = 0.0;

    public Effector() {
        intakeLeft.setControl(new Follower(intakeRight.getDeviceID(), true));

        effectorLeft.setNeutralMode(NeutralModeValue.Brake);
        effectorRight.setNeutralMode(NeutralModeValue.Brake);

        effectorTimer = new Timer();

        effectorLeft.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.effector.EFFECTOR_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.effector.EFFECTOR_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));

        effectorRight.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.effector.EFFECTOR_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.effector.EFFECTOR_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));
        
        intakeLeft.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.effector.EFFECTOR_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.effector.EFFECTOR_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));
        
        intakeRight.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.effector.EFFECTOR_STATOR_CURRENT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.effector.EFFECTOR_SUPPLY_CURRENT)
            .withSupplyCurrentLimitEnable(true));            

        effectorLeft.getConfigurator().apply( new Slot0Configs()
            .withKP(Constants.effector.P_EFFECTOR)
            .withKI(Constants.effector.I_EFFECTOR)
            .withKD(Constants.effector.D_EFFECTOR)
            .withKG(Constants.effector.G_EFFECTOR)
            .withKS(Constants.effector.S_EFFECTOR)
            .withKV(Constants.effector.V_EFFECTOR)
            .withKA(Constants.effector.A_EFFECTOR));
            
        effectorRight.getConfigurator().apply( new Slot0Configs()
            .withKP(Constants.effector.P_EFFECTOR)
            .withKI(Constants.effector.I_EFFECTOR)
            .withKD(Constants.effector.D_EFFECTOR)
            .withKG(Constants.effector.G_EFFECTOR)
            .withKS(Constants.effector.S_EFFECTOR)
            .withKV(Constants.effector.V_EFFECTOR)
            .withKA(Constants.effector.A_EFFECTOR));

        intakeLeft.getConfigurator().apply( new Slot0Configs()
            .withKP(Constants.intake.P_INTAKE)
            .withKI(Constants.intake.I_INTAKE)
            .withKD(Constants.intake.D_INTAKE)
            .withKG(Constants.intake.G_INTAKE)
            .withKS(Constants.intake.S_INTAKE)
            .withKV(Constants.intake.V_INTAKE)
            .withKA(Constants.intake.A_INTAKE));

        intakeRight.getConfigurator().apply( new Slot0Configs()
            .withKP(Constants.intake.P_INTAKE)
            .withKI(Constants.intake.I_INTAKE)
            .withKD(Constants.intake.D_INTAKE)
            .withKG(Constants.intake.G_INTAKE)
            .withKS(Constants.intake.S_INTAKE)
            .withKV(Constants.intake.V_INTAKE)
            .withKA(Constants.intake.A_INTAKE));
        
        effectorLeft.getConfigurator().apply(new VoltageConfigs()
            .withPeakForwardVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE)));
            
        effectorRight.getConfigurator().apply(new VoltageConfigs()
            .withPeakForwardVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE)));
            
        intakeLeft.getConfigurator().apply(new VoltageConfigs()
            .withPeakForwardVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE)));
            
        intakeRight.getConfigurator().apply(new VoltageConfigs()
            .withPeakForwardVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(Constants.effector.EFFECTOR_PEAK_VOLTAGE)));
        
        effectorLeft.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(30 * Constants.MASTER_SPEED_MULTIPLIER))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100 * Constants.MASTER_SPEED_MULTIPLIER)));

        effectorRight.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(10 * Constants.MASTER_SPEED_MULTIPLIER))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100 * Constants.MASTER_SPEED_MULTIPLIER)));
    }

    public void bumpRotations(double rotations) {
        double currentRotL = effectorLeft.getPosition().getValueAsDouble();
        double currentRotR = effectorRight.getPosition().getValueAsDouble();

        effectorLeft.setControl(m_positionVoltage.withPosition(currentRotL + rotations));
        effectorRight.setControl(m_positionVoltage.withPosition(currentRotR - rotations));
    }

    /**
     * Returns a Command that spins both wheels at a constant speed (in RPS)
     * until the left wheel has turned the given number of rotations.
     *
     * @param rotations How many rotations to spin
     * @param speedRPS Speed in rotations-per-second
     */
    public Command bumpSpeedRotations(double rotations, double speedRPS) {
        final double target = Math.abs(rotations);
        final double direction = Math.signum(rotations);
        return sequence(
            // 1) capture starting position
            Commands.runOnce(() -> m_initialLeftRot = effectorLeft.getPosition().getValueAsDouble(), this),
            // 2) spin at constant speed until we've turned enough
            Commands.run(() -> {
                double vel = direction * speedRPS * Constants.MASTER_SPEED_MULTIPLIER;
                effectorLeft.setControl(m_velocityVoltage.withVelocity(vel));
                effectorRight.setControl(m_velocityVoltage.withVelocity(-vel));
            }, this)
            .until(() ->
                Math.abs(effectorLeft.getPosition().getValueAsDouble() - m_initialLeftRot) >= target
            ),
            // 3) stop motors
            Commands.runOnce(() -> {
                effectorLeft.set(0);
                effectorRight.set(0);
            }, this)
        );
    }

    /**
     * Stops all intake and effector motors immediately.
     */
    public void stopIntake() {
        effectorLeft.set(0);
        effectorRight.set(0);
  //      intakeRight.set(0);
        intakeLeft.set(0);
        return;

    }

    public void startIntake() {
        // Turn on intake and effector wheels
        effectorLeft.setControl(m_velocityVoltage.withVelocity(40 * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRight.setControl(m_velocityVoltage.withVelocity(-40 * Constants.MASTER_SPEED_MULTIPLIER));

    //    intakeRight.setControl(m_velocityVoltage.withVelocity(-20 * Constants.masterSpeedMultiplier));
        intakeLeft.setControl(m_velocityVoltage.withVelocity(20 * Constants.MASTER_SPEED_MULTIPLIER));
        return;
    }

    public void startLock() {
        // Turn on intake
        effectorLeft.setControl(m_velocityVoltage.withVelocity(15 * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRight.setControl(m_velocityVoltage.withVelocity(-15 * Constants.MASTER_SPEED_MULTIPLIER));
        return;

    }

    public void startOutTake() {
        // Turn on intake
        effectorLeft.setControl(m_velocityVoltage.withVelocity(40 * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRight.setControl(m_velocityVoltage.withVelocity(-40 * Constants.MASTER_SPEED_MULTIPLIER));
        return;

    }

    public boolean isCoralDetected() {
        return intakeSensor.getMeasurement().distance_mm < 10;
    }

    public boolean isCoralNotDetected() {
        return intakeSensor.getMeasurement().distance_mm > 10;
    }


    public static void outtakeUntilDetected() {
        if (Elevator.getPosition() < 5) {
            asymmetricalOuttake(null, null);
        }
        else {
            while (intakeSensor.getMeasurement().distance_mm < 10) {
            symmetricalOuttake(null);
            }
        }
        effectorLeft.set(0);
        effectorRight.set(0);
        effectorTimer.stop();
        effectorTimer.reset();
        return;

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
            effectorLeft.setControl(m_velocityVoltage.withVelocity(motorSpeed * Constants.MASTER_SPEED_MULTIPLIER));
            effectorRight.setControl(m_velocityVoltage.withVelocity(-motorSpeed * Constants.MASTER_SPEED_MULTIPLIER));
        }
        effectorLeft.set(0);
        effectorRight.set(0);
        effectorTimer.stop();
        effectorTimer.reset();
        return;
    }

    public static void asymmetricalOuttake(Double velocityLeft, Double velocityRight) {
        System.out.println(effectorLeft.getPosition().getValueAsDouble());
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
            effectorLeft.setControl(m_velocityVoltage.withVelocity(motorSpeedL));
            effectorRight.setControl(m_velocityVoltage.withVelocity(-motorSpeedR));
        }

        effectorLeft.set(0);
        effectorRight.set(0);
        effectorTimer.stop();
        effectorTimer.reset();
        return;
    }

    public static void manualControl(double velocityLeft, Double velocityRight) {
        if (velocityRight == null) {
            velocityRight = -velocityLeft;
        }
        effectorLeft.setControl(m_velocityVoltage.withVelocity(velocityLeft * Constants.MASTER_SPEED_MULTIPLIER));
        effectorRight.setControl(m_velocityVoltage.withVelocity(velocityRight * Constants.MASTER_SPEED_MULTIPLIER));

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
        return;

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
        return;

    }

    public static void toggleAlgae() {
        if (isAlgaeOut) {
            algaeEffectorDown();
        }
        else {
            algaeEffectorUp(null);
        }
        return;

    }
}
