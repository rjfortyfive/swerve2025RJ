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

/**
 * RobotContainer is the central hub for robot configuration.
 * 
 * This class:
 * - Creates and configures all subsystems (drivetrain, elevator, effector, intake, etc.)
 * - Sets up all command bindings for driver and operator controllers
 * - Configures default commands that run when no other command requires the subsystem
 * - Registers named commands for use in PathPlanner autonomous routines
 * - Configures AutoBuilder for PathPlanner autonomous path following
 * 
 * The robot uses a command-based architecture where all robot actions are commands
 * that are bound to buttons/triggers and scheduled via the CommandScheduler.
 */
public class RobotContainer {
        
        /** Autonomous command chooser displayed on SmartDashboard */
        private final SendableChooser<Command> autoChooser;

        /** Maximum translational speed in meters per second (from TunerConstants) */
        public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        
        /** Maximum angular velocity in radians per second (1.25 rotations/sec) */
        public static double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond) 
                * Constants.MASTER_DRIVE_MULTIPLIER;

        /**
         * Field-centric drive request with 1% deadband on both translation and rotation.
         * Uses velocity control mode for smooth, responsive driving.
         */
        public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.01)
                        .withRotationalDeadband(MaxAngularRate * 0.01)
                        .withDriveRequestType(DriveRequestType.Velocity);
        
        /** Brake mode request for drivetrain (currently unused but available) */
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        /** Telemetry logger for drivetrain state */
        private final Telemetry logger = new Telemetry(MaxSpeed);

        /**
         * Driver controller selection flag.
         * true = Xbox controller, false = flight joystick
         * Change this value to switch between controller types
         */
        public static final boolean useXboxForDriver = true;

        // Controller instances - only the appropriate one is used based on useXboxForDriver
        /** Flight joystick on port 0 (used if useXboxForDriver is false) */
        public static final CommandJoystick DriverJoystick = new CommandJoystick(0);
        /** Xbox controller on port 0 (used if useXboxForDriver is true) */
        public static final CommandXboxController DriverXbox = new CommandXboxController(0);
        
        /** Operator Xbox controller on port 1 (controls mechanisms) */
        public static final CommandXboxController OperatorController = new CommandXboxController(1);
        /** Button panel on port 2 (custom button board for elevator/scoring) */
        public static final CommandJoystick buttonPanel = new CommandJoystick(2);

        // Subsystem instances
        /** Swerve drivetrain subsystem (4-wheel swerve with CANcoder absolute encoders) */
        public final static CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
        /** Elevator subsystem (moves scoring mechanism to different heights) */
        public final Elevator m_elevator = new Elevator();
        /** Effector subsystem (coral scoring wheels and algae removal mechanism) */
        public final Effector m_effector = new Effector();
        /** Intake subsystem (funnel intake for picking up coral) */
        public final Intake m_intake = new Intake();
        /** Hang subsystem (endgame climbing mechanism) */
        public final Hang m_hang = new Hang();
        /** Vision subsystem (PhotonVision for pose estimation via AprilTags) */
        public final Vision m_vision = new Vision();
        /** Lights subsystem (addressable LED strips for alliance indication) */
        public final Lights m_lights = new Lights();

        /**
         * Constructor - sets up the entire robot.
         * Configures PathPlanner AutoBuilder, binds all commands to controllers,
         * sets default commands, and registers named commands for autonomous.
         */
        public RobotContainer() {
                // Configure PathPlanner AutoBuilder for autonomous path following
                configureAutoBuilder();

                // Set up all button/trigger bindings for driver and operator
                configureBindings();
                
                // Configure default commands that run when no other command requires the subsystem
                configureDefaultCommands();
                
                // Register named commands for use in PathPlanner autonomous routines
                configureNamedCommands();

                // Build autonomous chooser and add to SmartDashboard
                autoChooser = AutoBuilder.buildAutoChooser("");
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        /**
         * Configures all command bindings for driver and operator controllers.
         * 
         * Button Panel bindings:
         * - L1/L2/L3/L4: Move elevator to scoring heights
         * - Coral IN: Intake coral from ground
         * - Coral OUT: Smart scoring (auto-selects L1 asymmetric or L2/L3/L4 based on elevator)
         * 
         * Operator Controller bindings:
         * - B: Move elevator to intake position
         * - X: Coral intake command
         * - Y: Move elevator to zero (bottom)
         * - D-Pad Up/Down: Move algae effector
         * - A: Asymmetric effector outtake for L1 scoring
         * - Left Bumper + Triggers: Hang mechanism control
         * 
         * Driver Controller bindings (Xbox or Joystick):
         * - Default command: Field-centric driving
         * - Bumpers: Strafe to AprilTag alignment (MakeGoToTag)
         * - Triggers (Xbox only): Coral station alignment
         */
        private void configureBindings() {
                // Elevator level buttons on button Panel
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
                // Smart scoring: automatically selects L1 asymmetric or L2/L3/L4 scoring based on elevator position
                buttonPanel.button(Constants.buttonPanel.coral.OUT)
                                .whileTrue(SmartScore.create(m_elevator, m_effector));
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
                // Register telemetry callback for drivetrain state logging
                m_drivetrain.registerTelemetry(logger::telemeterize);
                
                // Strafe and alignment commands - different bindings based on driver controller type
                // These are disabled if MASTER_NERF is true (for testing/troubleshooting)
                if (!Constants.MASTER_NERF) {
                        if (useXboxForDriver) {
                                // Strafe commands on Xbox controller bumpers
                                // Strafe Left: left bumper
                                DriverXbox.leftBumper()
                                        .whileTrue(new MakeGoToTag(
                                                m_drivetrain,
                                                m_vision,
                                                tagSide.LEFT,
                                                0.165,
                                                0.345
                                        ));

                                // Strafe Right: right bumper
                                DriverXbox.rightBumper()
                                        .whileTrue(new MakeGoToTag(
                                                m_drivetrain,
                                                m_vision,
                                                tagSide.RIGHT,
                                                0.197,
                                                0.345
                                        ));

                                // Coral Station Align commands on Xbox controller triggers
                                // Left trigger: align to LEFT coral station
                                new Trigger(() -> DriverXbox.getLeftTriggerAxis() > 0.25)
                                        .whileTrue(new CoralStationAlign(
                                                m_drivetrain,
                                                CoralStationAlign.CoralStationSide.LEFT
                                        ));

                                // Right trigger: align to RIGHT coral station
                                new Trigger(() -> DriverXbox.getRightTriggerAxis() > 0.25)
                                        .whileTrue(new CoralStationAlign(
                                                m_drivetrain,
                                                CoralStationAlign.CoralStationSide.RIGHT
                                        ));
                        } else {
                                // Strafe commands on joystick buttons
                                // Strafe Right: runs while button is held
                                DriverJoystick.button(Constants.Joystick.STRAFE_RIGHT)
                                        .whileTrue(new MakeGoToTag(
                                                m_drivetrain,
                                                m_vision,
                                                tagSide.RIGHT,
                                                0.197,
                                                0.345
                                        ));

                                // Strafe Left: runs while button is held
                                DriverJoystick.button(Constants.Joystick.STRAFE_LEFT)
                                        .whileTrue(new MakeGoToTag(
                                                m_drivetrain,
                                                m_vision,
                                                tagSide.LEFT,
                                                0.165,
                                                0.345
                                        ));
                        }
                }
        }

        /**
         * Registers named commands for use in PathPlanner autonomous routines.
         * 
         * These commands can be referenced by name in PathPlanner GUI when building autonomous paths.
         * Available named commands:
         * - "scoreL1Coral": Scores coral at L1 height with asymmetric outtake
         * - "scoreL4Coral": Scores coral at L4 height using ScoreL4L3L2 command
         * - "startIntakeCoral": Moves elevator to intake position and starts effector
         * - "raiseL2": Raises elevator to L2+5 rotations
         * - "intakeCoral": Full coral intake sequence
         */
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

        /**
         * Configures default commands that run continuously when no other command
         * requires the subsystem.
         * 
         * Default commands:
         * - Drivetrain: Field-centric driving from driver controller (Xbox or joystick)
         * - Effector: Manual control via operator controller triggers (outtake/intake)
         * - Lights: Continuously display alliance colors (purple/gold)
         */
        private void configureDefaultCommands() {
                // Default command for driving - switches between joystick and Xbox controller
                // based on useXboxForDriver flag
                if (useXboxForDriver) {
                        // Drive with Xbox controller: Left stick for translation, Right stick X for rotation
                        m_drivetrain.setDefaultCommand(
                                m_drivetrain.applyRequest(() -> 
                                        drive.withVelocityX(-DriverXbox.getLeftY() * MaxSpeed)
                                             .withVelocityY(-DriverXbox.getLeftX() * MaxSpeed)
                                             .withRotationalRate(-DriverXbox.getRightX() * MaxAngularRate)
                                )
                        );
                } else {
                        // Drive with flight joystick
                        m_drivetrain.setDefaultCommand(
                                m_drivetrain.applyRequest(() -> 
                                        drive.withVelocityX(DriverJoystick.getY() * MaxSpeed)
                                             .withVelocityY(DriverJoystick.getX() * MaxSpeed)
                                             .withRotationalRate(-DriverJoystick.getTwist() * MaxAngularRate)
                                )
                        );
                }

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
        /**
         * Configures PathPlanner AutoBuilder for autonomous path following.
         * 
         * AutoBuilder allows PathPlanner to generate autonomous paths that the robot
         * can follow. This configuration tells PathPlanner:
         * - How to get the robot's current pose
         * - How to reset the robot's pose
         * - How to get the robot's current speeds
         * - How to apply desired speeds to the drivetrain
         * - What PID controllers to use for path following
         * - Whether to mirror paths based on alliance color
         * 
         * Loads configuration from PathPlanner GUI settings file.
         */
        private void configureAutoBuilder() {
            try {
                // Load PathPlanner configuration from GUI settings
                var config = RobotConfig.fromGUISettings();
        
                AutoBuilder.configure(
                    // 1) Robot pose supplier - provides current robot pose for path following
                    () -> m_drivetrain.getState().Pose,
        
                    // 2) Pose reset function - allows PathPlanner to reset robot pose
                    (pose) -> m_drivetrain.resetPose(pose),
        
                    // 3) Robot speeds supplier - provides current velocity for path following
                    () -> m_drivetrain.getState().Speeds,
        
                    // 4) Apply chassis speeds - converts desired speeds to motor commands
                    // Also applies feedforward forces for improved tracking
                    (speeds, feedforwards) ->
                        m_drivetrain.setControl(
                                m_drivetrain.getPathApplyRobotSpeeds()
                                        .withSpeeds(speeds)
                                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                        ),
        
                    // 5) Holonomic drive controller - PID controllers for X/Y translation and rotation
                    // Translation: P=10, I=0, D=0
                    // Rotation: P=5, I=0, D=0
                    new PPHolonomicDriveController(
                        new PIDConstants(10, 0, 0),
                        new PIDConstants(5, 0, 0)
                    ),
        
                    // 6) PathPlanner GUI configuration (robot dimensions, max speeds, etc.)
                    config,
        
                    // 7) Alliance mirroring - mirror paths for red alliance
                    () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
        
                    // 8) Subsystem reference - drivetrain subsystem for path following
                    m_drivetrain
                );
        
            } catch (Exception ex) {
                DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace()
                );
            }
        }
                        
        

        /**
         * Gets the currently selected autonomous command from the chooser.
         * Called by Robot.autonomousInit() to start the selected autonomous routine.
         * 
         * @return The autonomous command selected on SmartDashboard, or null if none selected
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
