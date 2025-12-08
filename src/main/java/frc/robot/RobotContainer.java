// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.generated.TunerConstants;
import frc.robot.util.TagUtils.tagSide;

public class RobotContainer {
        
        private final SendableChooser<Command> autoChooser;

        public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                // desired
                                                                                                // top
                                                                                                // speed
        public static double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond) * Constants.MASTER_DRIVE_MULTIPLIER; // 3/4 of a rotation per
                                                                                               // second
                                                                                               // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10%
                                                                                                     // deadband
                        .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        public static final CommandJoystick DriverController = new CommandJoystick(0);
        public static final CommandXboxController OperatorController = new CommandXboxController(1);
        public static final CommandJoystick buttonPanel = new CommandJoystick(2);

        public final static CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
        public final Elevator m_elevator = new Elevator();
        public final Effector m_effector = new Effector();
        public final Intake m_intake = new Intake();
        public final Hang m_hang = new Hang();
        public final Vision m_vision = new Vision();
        public final Lights m_lights = new Lights();

        public RobotContainer() {
                configureAutoBuilder();

                configureBindings();
                configureDefaultCommands();
                configureNamedCommands();

                
                autoChooser = AutoBuilder.buildAutoChooser("");
                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        private void configureBindings() {
                // Elevator level buttons on button Board
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
                // Intake and Reef Score on Button Board
                buttonPanel.button(Constants.buttonPanel.coral.IN)
                                .onTrue(new CoralIntake(m_elevator, m_effector, m_intake));
                buttonPanel.button(Constants.buttonPanel.coral.OUT)
                                .onTrue(new ScoreL4L3L2(m_effector));
                // Elevator To Intake Position                                
                OperatorController.b().onTrue(
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> m_elevator
                                                                .toPosition(Constants.elevator.level.L1 + 2))));
                // Coral Intake Command
                OperatorController.x()
                                .onTrue(new CoralIntake(m_elevator, m_effector, m_intake));
                // Elevator to zero position
                OperatorController.y()
                                .onTrue(new InstantCommand(() -> m_elevator.toPosition(0)));
                // Algae Effector Up and Down on D-Pad
                OperatorController.povUp()
                                .whileTrue(new StartEndCommand(() -> m_effector.moveAlgaeEffector(1), 
                                                                () -> m_effector.stopAlgaeEffector(), m_effector));
                OperatorController.povDown()
                                .whileTrue(new StartEndCommand(() -> m_effector.moveAlgaeEffector(-1), 
                                                                () -> m_effector.stopAlgaeEffector(), m_effector));
                // Asymmetric Effector outtake for L1
                OperatorController.a()
                                .whileTrue(new StartEndCommand(() -> m_effector.start(20.0, 6.0), 
                                                                () -> m_effector.stop(), m_effector));
                // Hang control triggers: Only when left bumper is held and a trigger is pressed
                new Trigger(() -> OperatorController.leftBumper().getAsBoolean()
                                && OperatorController.getRightTriggerAxis() > 0.25)
                                .whileTrue(new StartEndCommand(() -> m_hang.start(100), () -> m_hang.stop(), m_hang, m_drivetrain));
                new Trigger(() -> OperatorController.leftBumper().getAsBoolean()
                                && OperatorController.getLeftTriggerAxis() > 0.25)
                                .whileTrue(new StartEndCommand(() -> m_hang.start(-100), () -> m_hang.stop(), m_hang, m_drivetrain));
                // reset the field-centric heading on middle button press
                // joystick.button(2).onTrue(drivetrain.runOnce(() ->
                // drivetrain.seedFieldCentric()));
                m_drivetrain.registerTelemetry(logger::telemeterize);
                // Button commands and stick-based triggers for strafeRight and strafeLeft
                if (!Constants.MASTER_NERF) {
                        // Strafe Right: schedule and track the command, only one at a time
                        DriverController.button(Constants.Joystick.STRAFE_RIGHT)
                                .onTrue(new MakeGoToTag(
                                        m_drivetrain,
                                        m_vision,
                                        tagSide.RIGHT,
                                        0.197,
                                        0.345
                                ));

                        // Strafe Left: schedule and track the command, only one at a time
                        DriverController.button(Constants.Joystick.STRAFE_LEFT)
                                .onTrue(new MakeGoToTag(
                                        m_drivetrain,
                                        m_vision,
                                        tagSide.LEFT,
                                        0.165,
                                        0.345
                                ));
                }
        }

        private void configureNamedCommands() {
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
                                                                                Constants.elevator.level.L4),
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
                                                                                Constants.elevator.level.INTAKE),
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

        }

        private void configureDefaultCommands() {

                // Default command for driving with joystick
                m_drivetrain.setDefaultCommand(
                        m_drivetrain.applyRequest(() -> 
                                drive.withVelocityX(DriverController.getY() * MaxSpeed)
                                     .withVelocityY(DriverController.getX() * MaxSpeed)
                                     .withRotationalRate(-DriverController.getTwist() * MaxAngularRate)
                        )
                );

                // Default command for effector control with Xbox triggers                                              
                m_effector.setDefaultCommand(
                        new RunCommand(() -> {
                            double lt = OperatorController.getLeftTriggerAxis();
                            double rt = OperatorController.getRightTriggerAxis();
                    
                            if (rt > 0.1) {
                                m_effector.start(70 * rt);  // Outtake
                            } else if (lt > 0.1) {
                                m_effector.start(-70 * lt); // Intake
                            } else {
                                m_effector.stop();
                            }
                        }, m_effector)
                    );
                // Default command for lights
                m_lights.setDefaultCommand(new RunCommand(() -> m_lights.lightsOn(Constants.lights.purpleGoldStep), m_lights));
        }
        private void configureAutoBuilder() {
            try {
                var config = RobotConfig.fromGUISettings();
        
                AutoBuilder.configure(
                    // 1) Robot pose supplier (EXACT same as before)
                    () -> m_drivetrain.getState().Pose,
        
                    // 2) Pose reset function (EXACT same)
                    (pose) -> m_drivetrain.resetPose(pose),
        
                    // 3) Robot speeds supplier (EXACT same)
                    () -> m_drivetrain.getState().Speeds,
        
                    // 4) Apply chassis speeds + feedforwards (EXACT same logic)
                    (speeds, feedforwards) ->
                        m_drivetrain.setControl(
                                m_drivetrain.getPathApplyRobotSpeeds()
                                        .withSpeeds(speeds)
                                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                        ),
        
                    // 5) Holonomic controller (EXACT same PID constants)
                    new PPHolonomicDriveController(
                        new PIDConstants(10, 0, 0),
                        new PIDConstants(5, 0, 0)
                    ),
        
                    // 6) PathPlanner GUI config (EXACT same)
                    config,
        
                    // 7) Alliance mirroring (EXACT same)
                    () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
        
                    // 8) Subsystem instance reference (EXACT same meaning as "this" before)
                    m_drivetrain
                );
        
            } catch (Exception ex) {
                DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace()
                );
            }
        }
                        
        

        public Command getAutonomousCommand() {
                // return Commands.print("No autonomous command configured");
                return autoChooser.getSelected();
        }
}
