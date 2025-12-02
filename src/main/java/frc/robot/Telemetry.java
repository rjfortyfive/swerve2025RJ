package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.DoublePublisher;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * 2025+ Compliant Telemetry class.
 * - Uses ONLY NT Struct publishers (no deprecated raw arrays)
 * - Logs swerve module data + drivetrain state
 * - Does NOT publish a robotPose (Robot.java handles Field2d)
 * - AdvantageScope-safe
 */
public class Telemetry {

    private final double maxSpeed;

    public Telemetry(double maxSpeed) {
        this.maxSpeed = maxSpeed;
        SignalLogger.start();
    }

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // ----------------------------
    // NetworkTables Publishers
    // ----------------------------

    private final NetworkTable driveStateTable =
        inst.getTable("DriveState");

    private final StructPublisher<Pose2d> drivePose =
        driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();

    private final StructPublisher<ChassisSpeeds> driveSpeeds =
        driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();

    private final StructArrayPublisher<SwerveModuleState> driveModuleStates =
        driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();

    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets =
        driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();

    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions =
        driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();

    private final DoublePublisher driveTimestamp =
        driveStateTable.getDoubleTopic("Timestamp").publish();

    private final DoublePublisher driveOdometryFrequency =
        driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    // ----------------------------
    // Mechanism2D Visualization
    // ----------------------------

    private final Mechanism2d[] moduleMechs = {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };

    private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
        moduleMechs[0].getRoot("Root", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechs[1].getRoot("Root", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechs[2].getRoot("Root", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechs[3].getRoot("Root", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };

    private final MechanismLigament2d[] moduleDirections = new MechanismLigament2d[] {
        moduleMechs[0].getRoot("RootDir", 0.5, 0.5)
            .append(new MechanismLigament2d("Dir", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechs[1].getRoot("RootDir", 0.5, 0.5)
            .append(new MechanismLigament2d("Dir", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechs[2].getRoot("RootDir", 0.5, 0.5)
            .append(new MechanismLigament2d("Dir", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechs[3].getRoot("RootDir", 0.5, 0.5)
            .append(new MechanismLigament2d("Dir", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    // ----------------------------
    // Main Telemetry Function
    // ----------------------------

    public void telemeterize(SwerveDriveState state) {

        // -------- Publish NT Struct Data --------
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        // -------- SignalLogger (CSV Log) --------
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "sec");

        // -------- Module Mechanism2D Display --------
        for (int i = 0; i < 4; i++) {
            var st = state.ModuleStates[i];

            moduleSpeeds[i].setAngle(st.angle);
            moduleDirections[i].setAngle(st.angle);

            // Scale speed bar by % of max speed
            moduleSpeeds[i].setLength(
                st.speedMetersPerSecond / (2 * maxSpeed)
            );

            SmartDashboard.putData("Module " + i, moduleMechs[i]);
        }
    }
}
