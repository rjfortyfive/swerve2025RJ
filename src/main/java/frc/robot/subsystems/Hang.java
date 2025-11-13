package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hang extends SubsystemBase{
    public static TalonFX hangMotor = new TalonFX(Constants.hang.hangMotor);
    public final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

    public static void activateHang(Boolean reverseDirection) {
        if (reverseDirection == false) {
            System.out.println("Moving climber up");
            hangMotor.setControl(m_velocityVoltage.withVelocity(100 * Constants.masterSpeedMultiplier));
        }
        else {
            System.out.println("Moving climber down");
            hangMotor.setControl(m_velocityVoltage.withVelocity(-100 * Constants.masterSpeedMultiplier));
        }
    }

    public static void stopHang() {
        hangMotor.setControl(new StaticBrake());
    }

    public static void brakeHang() {
        hangMotor.setControl(new StaticBrake());
    }


    public static void configureHang() {
        TalonFXConfiguration hangConfig = new TalonFXConfiguration();
        hangConfig.Voltage.withPeakForwardVoltage(Volts.of(12 * Constants.masterVoltageMultiplier)).withPeakReverseVoltage(Volts.of(-8 * Constants.masterVoltageMultiplier));
        hangConfig.Slot0.kP = 1;
        hangConfig.Slot0.kI = 0;
        hangConfig.Slot0.kD = 0;
        hangConfig.Slot0.kS = 5;
    }

}
