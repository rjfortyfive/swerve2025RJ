// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.generated.TunerConstants;
import frc.robot.util.TagUtils;
import frc.robot.util.tagSide;

public class RobotContainer {
        // Tracks the currently targeted AprilTag ID for stick-based rotation commands
        private int mCurrentTargetTag = -1;
        // Timestamp of the last strafe button press
        private double mLastStrafeButtonTime = 0;
        // Tracks which side was chosen on the last strafe button press
        private tagSide mCurrentTargetSide = tagSide.LEFT;
        // Tracks the currently scheduled auto-align command for cancellation
        private Command mCurrentAutoAlignCommand = null;

        // 1) Separate tag IDs into upper and lower groups
        private static final List<Integer> kStation = List.of(1, 2, 12, 13);

        // 2) Find the closest tag ID to the robot (search both groups)
        private int getClosestTagId() {
                Pose2d robotPose = drivetrain.getPose();
                List<Integer> allTags = new ArrayList<>(Constants.Vision.kTags);
                // allTags.addAll(kBlueTags);
                return allTags.stream()
                                .min(Comparator.comparingDouble(id -> TagUtils.getTagPose2d(id)
                                                .map(p -> p.getTranslation().getDistance(robotPose.getTranslation()))
                                                .orElse(Double.MAX_VALUE)))
                                .orElse(Constants.Vision.kTags.get(0));
        }

        public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final SendableChooser<Command> autoChooser;
        public static SendableChooser<Integer> positionChooser;

        public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 1; // kSpeedAt12Volts
                                                                                                // desired
                                                                                                // top
                                                                                                // speed
        public static double MaxAngularRate = RotationsPerSecond.of(2.5).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                               // second
                                                                                               // max angular velocity

        /** Shared path-following constraints for all tag paths */
        public static final PathConstraints kPathConstraints = new PathConstraints(
                        Constants.Pathfinding.MaxSpeed,
                        Constants.Pathfinding.MaxAccel,
                        Units.degreesToRadians(Constants.Pathfinding.MaxRotSpeed),
                        Units.degreesToRadians(Constants.Pathfinding.MaxRotAccel));

        /* Setting up bindings for necessary control of the swerve drive platform */
        public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10%
                                                                                                     // deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        private final Telemetry logger = new Telemetry(MaxSpeed);
        public final Vision vision;

        public static final CommandJoystick joystick = new CommandJoystick(0);
        public static final CommandXboxController XboxController = new CommandXboxController(1);
        public static final CommandJoystick buttonPanel = new CommandJoystick(2);

        final DutyCycleOut m_leftRequest = new DutyCycleOut(0.0);
        final DutyCycleOut m_rightRequest = new DutyCycleOut(0.0);

        public Elevator m_elevator = new Elevator();
        public Effector m_effector = new Effector();
        public Intake m_intake = new Intake();
        public final Hang m_hang = new Hang();

        public RobotContainer() {
                vision = new Vision((pose, timestamp, stdDevs) -> drivetrain.addVisionMeasurement(pose, timestamp,
                                stdDevs));
                configureBindings();

                NamedCommands.registerCommand("scoreL1Coral",
                                new SequentialCommandGroup(
                                                new InstantCommand(
                                                                () -> m_elevator.toPosition(
                                                                                Constants.elevator.level.L1 + 2.0),
                                                                m_elevator),
                                                new InstantCommand(() -> m_effector.start(30, 10),
                                                                m_effector),
                                                new InstantCommand(
                                                                () -> m_elevator.toPosition(Constants.elevator.level.L1),
                                                                m_elevator)));

                NamedCommands.registerCommand("scoreL4Coral",
                                new SequentialCommandGroup(
                                                new InstantCommand(
                                                                () -> m_elevator.toPosition(
                                                                                Constants.elevator.level.L4 - 0.5),
                                                                m_elevator),

                                                new WaitCommand(.9),

                                                new ScoreL4L3L2(m_effector),
                                                new InstantCommand(
                                                                () -> m_elevator.toPosition(
                                                                                Constants.elevator.level.L1))));

                NamedCommands.registerCommand("startIntakeCoral",
                                sequence(
                                                // 1) move the elevator up to position intake
                                                new InstantCommand(
                                                                () -> m_elevator.toPosition(
                                                                                Constants.elevator.level.intake),
                                                                m_elevator),

                                                new InstantCommand(
                                                                () -> m_effector.start(40),
                                                                m_effector

                                                )));

                NamedCommands.registerCommand("raiseL2",
                                sequence(
                                                // 1) move the elevator up to L2
                                                new InstantCommand(
                                                                () -> m_elevator.toPosition(
                                                                                Constants.elevator.level.L2 + 5),
                                                                m_elevator)

                                ));
                NamedCommands.registerCommand("intakeCoral",
                                sequence(new CoralIntake(m_elevator, m_effector, m_intake)));

                autoChooser = AutoBuilder.buildAutoChooser("");
                SmartDashboard.putData("Auto Chooser", autoChooser);

                // Position chooser for autonomous starting positions
                positionChooser = new SendableChooser<>();
                positionChooser.setDefaultOption("Id 7", 1);
                positionChooser.addOption("Skip", 2);
                positionChooser.addOption("Id 8", 3);
                positionChooser.addOption("Id 8", 4);

                SmartDashboard.putData("Pose start", positionChooser);

        }

        private Command makeGoToTag(
                        int tagId,
                        tagSide side,
                        double offsetMeters,
                        double frontOffsetMeters) {
                // 1) compute the goal pose with your two offsets
                Pose2d goal = TagUtils.computeTagAdjacencyPose(
                                tagId,
                                side,
                                offsetMeters,
                                frontOffsetMeters);
                Logger.debug("computed adjacency goal: {}", goal);

                PathConstraints constraints = kPathConstraints;
                // 2) build your PathPlanner command as before
                Command pathCmd = AutoBuilder.pathfindToPose(goal, constraints, 0.0);

                // Canceller: finishes as soon as any joystick movement > 20%
                Command cancelOnStick = waitUntil(() -> Math.abs(joystick.getY()) > 0.2 ||
                        Math.abs(joystick.getX()) > 0.2 ||
                        Math.abs(joystick.getTwist()) > 0.2);

                // Race them: whichever ends first wins and cancels the other
                return race(pathCmd, cancelOnStick)
                    .andThen(new InstantCommand(() -> {
                        mCurrentAutoAlignCommand = null;
                    }));
        }

        private void configureBindings() {
                buttonPanel.button(Constants.buttonPanel.lift.L1)
                                .onTrue(new InstantCommand(() -> m_elevator.toPosition(Constants.elevator.level.L1)));
                buttonPanel.button(Constants.buttonPanel.lift.L2)
                                .onTrue(new InstantCommand(() -> m_elevator.toPosition(Constants.elevator.level.L2)));
                buttonPanel.button(Constants.buttonPanel.lift.L3)
                                .onTrue(new InstantCommand(() -> m_elevator.toPosition(Constants.elevator.level.L3)));
                buttonPanel.button(Constants.buttonPanel.lift.L4)
                                .onTrue(new InstantCommand(() -> m_elevator.toPosition(Constants.elevator.level.L4)));
                // buttonPanel.button(Constants.buttonPanel.algae.Lower)
                //                 .onTrue(new InstantCommand(() -> Sequences.removeL2Algae()));
                // buttonPanel.button(Constants.buttonPanel.algae.Upper)
                //                 .onTrue(new InstantCommand(() -> Sequences.removeL3Algae()));

                // toggle intake on/off each press
                buttonPanel.button(Constants.buttonPanel.coral.In)
                                .whileTrue(new CoralIntake(m_elevator, m_effector, m_intake));

                buttonPanel.button(Constants.buttonPanel.coral.Out)
                                .onTrue(new ScoreL4L3L2(m_effector));

                XboxController.a().onTrue(
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> m_elevator
                                                                .toPosition(Constants.elevator.level.L1 + 2))));

                XboxController.rightBumper().whileTrue(new RunCommand(
                                () -> m_effector.start(
                                                XboxController.getRawAxis(Constants.XboxController.axis.RightYAxis)
                                                                * 10)));

                XboxController.x()
                                .onTrue(new CoralIntake(m_elevator, m_effector, m_intake));

                XboxController.y()
                                .onTrue(new InstantCommand(() -> m_elevator.toPosition(0)));

                XboxController.povUp()
                                .whileTrue(new InstantCommand(() -> m_effector.algaeEffectorUp(1), m_effector));

                XboxController.povDown()
                                .whileTrue(new InstantCommand(() -> m_effector.algaeEffectorDown(1), m_effector));

                XboxController.a()
                                .whileTrue(new InstantCommand(() -> {
                                        m_effector.start(20.0, 6.0);
                                }, m_effector));

                // Hang control triggers: Only when left bumper is held and a trigger is pressed
                new Trigger(() -> XboxController.button(Constants.XboxController.bumper.Left).getAsBoolean()
                                && XboxController.getRawAxis(Constants.XboxController.axis.RightTrigger) > 0.25)
                                .whileTrue(new InstantCommand(() -> Hang.activateHang(false), drivetrain))
                                .onFalse(new InstantCommand(() -> Hang.stopHang(), drivetrain));

                new Trigger(() -> XboxController.button(Constants.XboxController.bumper.Left).getAsBoolean()
                                && XboxController.getRawAxis(Constants.XboxController.axis.LeftTrigger) > 0.25)
                                .whileTrue(new InstantCommand(() -> Hang.activateHang(true), drivetrain))
                                .onFalse(new InstantCommand(() -> Hang.stopHang(), drivetrain));

                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                        .withVelocityX(joystick.getY() * MaxSpeed * Constants.masterDriveMultiplier)
                                                        .withVelocityY(joystick.getX()  * MaxSpeed * Constants.masterDriveMultiplier)
                                                        .withRotationalRate(-joystick.getTwist()  * MaxAngularRate * Constants.masterDriveMultiplier)));
                
                m_effector.setDefaultCommand(new RunCommand(() -> {
                                double lt = XboxController.getLeftTriggerAxis();
                                double rt = XboxController.getRightTriggerAxis();
                                                            
                                        if (lt > 0.1) {
                                                m_effector.start(-0.5 * 70 * lt);
                                        } else if (rt > 0.1) {
                                                m_effector.start(0.5 * 70 * rt);
                                        } else {
                                                m_effector.start(0);
                                        }
                                        }, m_effector));
                                                                            


                // reset the field-centric heading on middle button press

                // joystick.button(2).onTrue(drivetrain.runOnce(() ->
                // drivetrain.seedFieldCentric()));

                drivetrain.registerTelemetry(logger::telemeterize);

                // Button commands

                // Button commands and stick-based triggers for strafeRight and strafeLeft
                if (!Constants.masterNerf) {
                        // Strafe Right: schedule and track the command, only one at a time
                        joystick.button(Constants.Joystick.strafeRight)
                                        .whileTrue(new RunCommand(() -> {

                                                int closest = getClosestTagId();

                                                mCurrentTargetSide = tagSide.RIGHT;

                                                if (mCurrentAutoAlignCommand != null) {
                                                        mCurrentAutoAlignCommand.cancel();
                                                        mCurrentAutoAlignCommand = null;
                                                }

                                                Command strafeRightCmd = makeGoToTag(closest, tagSide.RIGHT, 0.197,
                                                                0.345);

                                                strafeRightCmd.schedule();
                                                mCurrentAutoAlignCommand = strafeRightCmd;

                                                return;
                                        }, drivetrain));

                        // Strafe Left: schedule and track the command, only one at a time
                        joystick.button(Constants.Joystick.strafeLeft)
                                        .whileTrue(new RunCommand(() -> {
                                                int closest = getClosestTagId();
                                                mCurrentTargetTag = closest;
                                                mCurrentTargetSide = tagSide.LEFT;
                                                if (mCurrentAutoAlignCommand != null) {
                                                        mCurrentAutoAlignCommand.cancel();
                                                        mCurrentAutoAlignCommand = null;
                                                }
                                                Command strafeLeftCmd = makeGoToTag(closest, tagSide.LEFT, 0.165,
                                                                0.345);

                                                strafeLeftCmd.schedule();

                                                mCurrentAutoAlignCommand = strafeLeftCmd;
                                                return;
                                        }, drivetrain));

                        // Add a cancel binding
                        joystick.button(Constants.Joystick.Function1).onTrue(new InstantCommand(() -> {
                                if (mCurrentAutoAlignCommand != null) {
                                        mCurrentAutoAlignCommand.cancel();
                                        mCurrentAutoAlignCommand = null;
                                }
                        }));

                        // Path to the closest station
                        // joystick.button(2)
                        //                 .onTrue(new InstantCommand(() -> {
                        //                         pathToClosestStation().schedule();
                        //                 }, drivetrain));
                }
        }

        public Command getAutonomousCommand() {
                // return Commands.print("No autonomous command configured");
                return autoChooser.getSelected();
        }
}
