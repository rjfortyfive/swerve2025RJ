package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.Matrix;

public class Constants {
    public static final double MASTER_SPEED_MULTIPLIER = 1; // For troubleshooting/testing
    public static final double masterDriveMultiplier = 1; // For troubleshooting/testing Drivetrain
    public static final boolean masterNerf = false; // For troubleshooting/testing Drivetrain

    public static final double masterVoltageMultiplier = 1;

    // Right Camera
    public static final String CameraName1 = "EagleEye01";

    // Left Camera
    public static final String CameraName2 = "EagleEye02";

    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.

    // Right Cam
    public static final Transform3d RobotToCam1 = new Transform3d(new Translation3d(0.2786892, -0.2726416, 0.1499719),
            new Rotation3d(0, -0.3490659, 0.1745329));

    // Left Cam
    public static final Transform3d RobotToCam2 = new Transform3d(new Translation3d(0.2764772, 0.2724549, 0.1499719),
            new Rotation3d(0, -0.3490659, -0.1745329));

    // Andymark Field Layout
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(
            0.25, // σₓ: odometry may drift ±10 cm
            0.25, // σᵧ: same sideways
            Math.toRadians(.15) // σθ: roughly ±5° heading error
    );
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(
            0.18, // σₓ: vision ±0.5 cm
            0.18, // σᵧ: ±0.5 cm
            Math.toRadians(.025) // σθ: ±2°
    );
    public static final Matrix<N3, N1> kOdometryStdDevs = VecBuilder.fill(
            0.075, // 2 cm
            0.075, // 2 cm
            Math.toRadians(.1) // 2°
    );

    public static class Pathfinding {
        // max translation m/s
        public static final double MaxSpeed = 5;
        // max accel m/s²
        public static final double MaxAccel = 2.5;
        // max rot deg/s
        public static final double MaxRotSpeed = 600;
        // max rot accel deg/s²
        public static final double MaxRotAccel = 600;

    }

    public static class Vision {
        public static final String CameraName1 = Constants.CameraName1;
        public static final Transform3d RobotToCam1 = Constants.RobotToCam1;
        public static final String CameraName2 = Constants.CameraName2;
        public static final Transform3d RobotToCam2 = Constants.RobotToCam2;
        public static final AprilTagFieldLayout kTagLayout = Constants.kTagLayout;
        public static final Matrix<N3, N1> kSingleTagStdDevs = Constants.kSingleTagStdDevs;
        public static final Matrix<N3, N1> kMultiTagStdDevs = Constants.kMultiTagStdDevs;
        public static final Matrix<N3, N1> kOdometryStdDevs = Constants.kOdometryStdDevs;
        public static List<Integer> kTags = List.of();

        /** Odometry update rate in Hz for the SwerveDrivePoseEstimator */
        public static final double kOdometryUpdateHz = 250.0;

        /** PhotonVision pipeline index to use (0-based) */
        public static final int kPipelineIndex = 0;

    }

    public static final class Joystick {
        public static final int Function1 = 10;
        // public static final int Function2 = 9;
        public static final int strafeLeft = 6;
        public static final int strafeRight = 7 ;
        public static final int servoControl = 8;
        public static final int intakeResetButton = 9;
    }

    public static final class buttonPanel {
        public static final class lift {
            public static final int L1 = 3;
            public static final int L2 = 2;
            public static final int L3 = 4;
            public static final int L4 = 8;
        }

        public static final class coral {
            public static final int In = 1;
            public static final int Out = 7;
        }

        public static final class algae {
            public static final int Lower = 5;
            public static final int Upper = 9;
        }

        public static final class intake {
            public static final int intakeDropButton = 6;
            public static final int cancel = 10;

        }
    }

    public static final class XboxController {
        public static final class bumper {
            public static final int Left = 5;
            public static final int Right = 6;
        }

        public static final class button {
            public static final int A = 1;
            public static final int B = 2;
            public static final int X = 3;
            public static final int Y = 4;
            public static final int Start = 8;
            public static final int Window = 7;
        }

        public static final class axis {
            public static final int LeftXAxis = 0;
            public static final int LeftYAxis = 1;
            public static final int RightXAxis = 4;
            public static final int RightYAxis = 5;
            public static final int LeftTrigger = 2;
            public static final int RightTrigger = 3;
        }

        public static final class dpad {
            public static final int Up = 0;
            public static final int Right = 90;
            public static final int Down = 180;
            public static final int Left = 270;
        }
    }

    public static final class elevator {
        public static final int LIFT_LEFT_ID = 10;
        public static final int LIFT_RIGHT_ID = 11;

        public static final int LIFT_BOTTOM_SWITCH = 0;
        public static final int LIFT_TOP_SWITCH = 1;

        public static class level {
            public static double L1 = 0.05;
            public static double L2 = 15.5;
            public static double L3 = 37.5;
            public static double L4 = 73.0;
            public static double intake = 1.2;
            public static double activeLevel = 1;

        }

        public static final class algaeLevel {
            public static final double L2 = 30.0;
            public static final double L3 = 49.0;
        }
    }

    public static final class effector {
        public static final int EFFECTOR_LEFT_ID = 12;
        public static final int EFFECTOR_RIGHT_ID = 13;
        public static final double defaultVelocity = 15;
        public static final double scoreRotations = 4;
        public static final double scoreVelocity = 15;
  // We are using Motion Magic position control with voltage countrol output type,
  // so the PID parameters are in volts
        public static final double P_EFFECTOR = 1.0; 
        public static final double I_EFFECTOR = 0.0;
        public static final double D_EFFECTOR = 0.0; 
        public static final double G_EFFECTOR = 0.0;
        public static final double S_EFFECTOR = 1.0;
        public static final double V_EFFECTOR = 0.123;
        public static final double A_EFFECTOR = 0.0;

        public static final double EFFECTOR_STATOR_CURRENT = 80.0;
        public static final double EFFECTOR_SUPPLY_CURRENT = 40.0;

        public static final double EFFECTOR_PEAK_VOLTAGE = 8.0 * Constants.masterVoltageMultiplier;

    }

    public static final class intake {
        public static final int intakeRightID = 15;
        public static final int intakeLeftID = 16;
        public static final double lockRotations = -2.75;
        public static final double intakeRotations = 2.5;
        public static final double lockSpeedRPS = 25;
        public static final double P_INTAKE = 0.3;
        public static final double D_INTAKE = 0.0;
        public static final double I_INTAKE = 0.0;
        public static final double A_INTAKE = 0.0;
        public static final double G_INTAKE = 0.0;
        public static final double S_INTAKE = 0.0;
        public static final double V_INTAKE = 0.0;

        public static final double kIntakeStatorCurrent = 80.0;
        public static final double kIntakeSupplyCurrent = 40.0;

        public static final double INTAKE_PEAK_VOLTAGE = 8.0 * Constants.masterVoltageMultiplier;
    }

    public static final class hang {
        public static final int hangMotor = 14;
    }

    public static void setL4() {
        var driverStation = DriverStationJNI.getAllianceStation().toString();
        if (driverStation.contains("Blue")) {
            elevator.level.L4 = 72.5;

            Vision.kTags = List.of(17, 18, 19, 20, 21, 22);

            System.out.println("Setting blue side L4");
        } else {
            elevator.level.L4 = 73; // 74.0 for good wheels

            Vision.kTags = List.of(6, 7, 8, 9, 10, 11);

            System.out.println("Setting red side L4");
        }
    }

    public static final class lights {
        public static LEDPattern purple = LEDPattern.solid(Color.kPurple);
    }

}
