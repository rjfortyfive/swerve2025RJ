package frc.robot;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.path.PathConstraints;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;

public class Constants {
    public static final double MASTER_SPEED_MULTIPLIER = 1; // For troubleshooting/testing
    public static final double MASTER_DRIVE_MULTIPLIER = 1; // For troubleshooting/testing Drivetrain
    public static final boolean MASTER_NERF = false; // For troubleshooting/testing Drivetrain

    public static final double MASTER_VOLTAGE_MULTIPLIER = 1;

    public static class CanIDs {
        //Drivetrain on CANivore
        public static final int PIGEON_ID = 0;
        public static final int FRONT_LEFT_DRIVE_FX_ID = 1;
        public static final int FRONT_LEFT_STEER_FX_ID = 2;
        public static final int FRONT_LEFT_ENCODER_ID = 14;
        public static final int FRONT_RIGHT_DRIVE_FX_ID = 3;
        public static final int FRONT_RIGHT_STEER_FX_ID = 4;
        public static final int FRONT_RIGHT_ENCODER_ID = 16;
        public static final int BACK_LEFT_DRIVE_FX_ID = 5;
        public static final int BACK_LEFT_STEER_FX_ID = 6;
        public static final int BACK_LEFT_ENCODER_ID = 18;
        public static final int BACK_RIGHT_DRIVE_FX_ID = 7;
        public static final int BACK_RIGHT_STEER_FX_ID = 8;
        public static final int BACK_RIGHT_ENCODER_ID = 20;
        //Elevator on RIO
        public static final int ELEVATOR_LEFT_FX_ID = 10;
        public static final int ELEVATOR_RIGHT_FX_ID = 11;
        //Effector on RIO
        public static final int EFFECTOR_LEFT_FX_ID = 12;
        public static final int EFFECTOR_RIGHT_FX_ID = 13;
        public static final int EFFECTOR_LASER_ID = 2;
        //Intake on RIO
        public static final int INTAKE_RIGHT_FX_ID = 15;
        public static final int INTAKE_LEFT_FX_ID = 16;
        //Hang on RIO
        public static final int HANG_FX_ID = 14;


    }

    public static class Pathfinding {
        // max translation m/s
        public static final double MAX_SPEED = 5;
        // max accel m/s²
        public static final double MAX_ACCEL = 2.5;
        // max rot deg/s
        public static final double MAX_ROT_SPEED = 300;
        // max rot accel deg/s²
        public static final double MAX_ROT_ACCEL = 600;

        public static final PathConstraints kPathConstraints =
                new PathConstraints(
                        MAX_SPEED,
                        MAX_ACCEL,
                        Units.degreesToRadians(MAX_ROT_SPEED),
                        Units.degreesToRadians(MAX_ROT_ACCEL));

    }
// Bindings for joystick and Button Panel
    public static final class Joystick {
        public static final int FUNCTION_1 = 10;
        // public static final int Function2 = 9;
        public static final int STRAFE_LEFT = 6;
        public static final int STRAFE_RIGHT = 7 ;
    }

    public static final class buttonPanel {
        public static final class lift {
            public static final int L1 = 3;
            public static final int L2 = 2;
            public static final int L3 = 4;
            public static final int L4 = 8;
        }

        public static final class coral {
            public static final int IN = 1;
            public static final int OUT = 7;
        }

        public static final class algae {
            public static final int LOWER = 5;
            public static final int UPPER = 9;
        }

        public static final class intake {
            public static final int CANCEL = 10;

        }
    }
//Subsystem Constants

// Effector Constants
    public static final class effector {
        public static final double DEFAULT_VELOCITY = 15;
        public static final double SCORE_ROTATIONS = 4;
        public static final double SCORE_VELOCITY = 15;
  // We are using Motion Magic position control with voltage control output type,
  // so the PID parameters are in volts
        public static final double P_EFFECTOR = 1.0; 
        public static final double I_EFFECTOR = 0.0;
        public static final double D_EFFECTOR = 0.0; 
        public static final double G_EFFECTOR = 0.0;
        public static final double S_EFFECTOR = 1.0;
        public static final double V_EFFECTOR = 0.0;
        public static final double A_EFFECTOR = 0.0;

        public static final double EFFECTOR_STATOR_CURRENT = 80.0;
        public static final double EFFECTOR_SUPPLY_CURRENT = 40.0;

        public static final double EFFECTOR_PEAK_VOLTAGE = 8.0 * Constants.MASTER_VOLTAGE_MULTIPLIER;

        public static final double EFFECTOR_LEFT_CRUISE_VELOCITY = 30;
        public static final double EFFECTOR_RIGHT_CRUISE_VELOCITY = 10;
        public static final double EFFECTOR_ACCEL = 100;

        public static final double LOCK_ROTATIONS = -2.75;
    }
// Elevator Constants
    public static final class elevator {
        public static final int ELEVATOR_BOTTOM_SWITCH = 0;
        public static final double P_ELEVATOR = 1.5;
        public static final double I_ELEVATOR = 0.06;
        public static final double D_ELEVATOR = 0.01;
        public static final double G_ELEVATOR = 0.42;
        public static final double S_ELEVATOR = 0.3;
        public static final double V_ELEVATOR = 0.13;
        public static final double A_ELEVATOR = 0;

        public static final double ELEVATOR_STATOR_CURRENT = 120;
        public static final double ELEVATOR_SUPPLY_CURRENT = 60;

        public static final double ELEVATOR_CRUISE_VELOCITY = 90;
        public static final double ELEVATOR_ACCEL = 500;

        public static final double ELEVATOR_LOWER_LIMIT = 0;
        public static final double ELEVATOR_UPPER_LIMIT = 75;

        public static class level {
            public static double L1 = 0.05;
            public static double L2 = 15.5;
            public static double L3 = 37.5;
            public static double L4 = 73.0;
            public static double INTAKE = 1.2;
            public static double ACTIVE_LEVEL = 1;

        }

        public static final class algaeLevel {
            public static final double L2 = 30.0;
            public static final double L3 = 49.0;
        }
    }
// Hang Constants
    public static final class hang {
        public static final double P_HANG = 1;
        public static final double I_HANG = 0;
        public static final double D_HANG = 0;
        public static final double S_HANG = 5;

        public static final double HANG_STATOR_CURRENT = 80.0;
        public static final double HANG_SUPPLY_CURRENT = 40.0; 

        public static final double HANG_PEAK_FORWARD_VOLTAGE = 12.0 * Constants.MASTER_VOLTAGE_MULTIPLIER;
        public static final double HANG_PEAK_REVERSE_VOLTAGE = 8.0 * Constants.MASTER_VOLTAGE_MULTIPLIER;

    }
// Intake Constants
    public static final class intake {
        public static final int INTAKE_RIGHT_FX_ID = 15;
        public static final int INTAKE_LEFT_FX_ID = 16;
        public static final double INTAKE_ROTATIONS = 2.5;
        public static final double LOCK_SPEED_RPS = 25;
        public static final double P_INTAKE = 0.3;
        public static final double I_INTAKE = 0.0;
        public static final double D_INTAKE = 0.0;
        public static final double A_INTAKE = 0.0;
        public static final double G_INTAKE = 0.0;
        public static final double S_INTAKE = 0.0;
        public static final double V_INTAKE = 0.0;

        public static final double INTAKE_STATOR_CURRENT = 65.0;
        public static final double INTAKE_SUPPLY_CURRENT = 40.0;

        public static final double INTAKE_PEAK_VOLTAGE = 8.0 * Constants.MASTER_VOLTAGE_MULTIPLIER;
    }
// Lights Constants
    public static final class lights {
        public static Color purple = Color.fromHSV(135, 220, 170); // Uses OpenCV HSV values
        public static Color gold = Color.fromHSV(23, 242, 214);
        public static LEDPattern purpleGoldStep = LEDPattern.steps(Map.of(0, purple, 0.5, gold));
    }
// Vision Constants
    public static class Vision {
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        //right camera
        public static final String CAMERA_NAME_1 = "EagleEye01";
        public static final Transform3d ROBOT_TO_CAM_1 = new Transform3d(new Translation3d(0.2786892, -0.2726416, 0.1499719),
        new Rotation3d(0, -0.3490659, 0.1745329));
        //left camera
        public static final String CAMERA_NAME_2 = "EagleEye02";
        public static final Transform3d ROBOT_TO_CAM_2 = new Transform3d(new Translation3d(0.2764772, 0.2724549, 0.1499719),
        new Rotation3d(0, -0.3490659, -0.1745329));
        public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeAndyMark);
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(
            0.45, // σₓ: odometry may drift ±10 cm
            0.45, // σᵧ: same sideways
            Units.degreesToRadians(12.0) // σθ: roughly ±5° heading error
    );
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(
            0.18, // σₓ: vision ±0.5 cm
            0.18, // σᵧ: ±0.5 cm
            Units.degreesToRadians(4.0) // σθ: ±2°
    );

        public static List<Integer> TAGS = List.of();

        /** Odometry update rate in Hz for the SwerveDrivePoseEstimator */
        public static final double ODOMETRY_UPDATE_HZ = 250.0;

        /** PhotonVision pipeline index to use (0-based) */
        public static final int PIPLINE_INDEX = 0;

    }
// Sets L4 and Tags based on alliance color
    public static void setL4() {
        var driverStation = DriverStationJNI.getAllianceStation().toString();
        if (driverStation.contains("Blue")) {
            elevator.level.L4 = 72.5;

            Vision.TAGS = List.of(17, 18, 19, 20, 21, 22);

            System.out.println("Setting blue side L4");
        } else {
            elevator.level.L4 = 73; // 74.0 for good wheels

            Vision.TAGS = List.of(6, 7, 8, 9, 10, 11);

            System.out.println("Setting red side L4");
        }
    }

}
