package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequenceCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.util.TagUtils.tagSide;

import frc.robot.subsystems.*;

import frc.robot.state.RobotState;
import frc.robot.state.RobotStateManager;

public class RobotContainer {

    // === State Machine ===
    public final RobotStateManager stateManager = new RobotStateManager();

    // === Auto ===
    private final SendableChooser<Command> autoChooser;
    public static SendableChooser<Integer> positionChooser;

    // === Drive limits ===
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static double MaxAngularRate = RotationsPerSecond.of(2.5).in(RadiansPerSecond);

    // === Requests ===
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01)
            .withRotationalDeadband(MaxAngularRate * 0.01)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // === Telemetry ===
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // === Driver controls ===
    public static final CommandJoystick driver = new CommandJoystick(0);
    public static final CommandXboxController operator = new CommandXboxController(1);
    public static final CommandJoystick panel = new CommandJoystick(2);

    // === Subsystems ===
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator(stateManager);
    public final Effector effector = new Effector(stateManager);
    public final Intake intake = new Intake(stateManager);
    public final Hang hang = new Hang(stateManager);
    public final Vision vision = new Vision();
    public final Lights lights = new Lights(stateManager);

    public RobotContainer() {

        configureBindings();
        configureDefaultCommands();
        configureNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        positionChooser = new SendableChooser<>();
        positionChooser.setDefaultOption("ID 7", 1);
        positionChooser.addOption("Skip", 2);
        positionChooser.addOption("ID 8", 3);
        positionChooser.addOption("ID 9", 4);

        SmartDashboard.putData("Pose start", positionChooser);

        drivetrain.registerTelemetry(logger::telemeterize);
    }


    // =======================================================================
    //                       STATE MACHINE INPUT BINDINGS
    // =======================================================================

    private void configureBindings() {

        // === Lift Levels (Operator Panel) ===
        panel.button(Constants.buttonPanel.lift.L1)
                .onTrue(new InstantCommand(() ->
                        stateManager.setState(RobotState.INTAKE_CORAL)));

        panel.button(Constants.buttonPanel.lift.L2)
                .onTrue(new InstantCommand(() ->
                        stateManager.setState(RobotState.SCORE_CORAL)));

        panel.button(Constants.buttonPanel.lift.L3)
                .onTrue(new InstantCommand(() ->
                        stateManager.setState(RobotState.SCORE_CORAL)));

        panel.button(Constants.buttonPanel.lift.L4)
                .onTrue(new InstantCommand(() ->
                        stateManager.setState(RobotState.SCORE_CORAL)));

        // === Coral Intake (Panel) ===
        panel.button(Constants.buttonPanel.coral.IN)
                .onTrue(new InstantCommand(() ->
                        stateManager.setState(RobotState.INTAKE_CORAL)));

        // === Coral Out (Panel) ===
        panel.button(Constants.buttonPanel.coral.OUT)
                .onTrue(new InstantCommand(() ->
                        stateManager.setState(RobotState.SCORE_CORAL)));

        // === Operator Controls ===
        operator.x().onTrue(
                new InstantCommand(() -> stateManager.setState(RobotState.INTAKE_CORAL)));

        operator.y().onTrue(
                new InstantCommand(() -> stateManager.setState(RobotState.IDLE)));

        operator.a().onTrue(
                new InstantCommand(() -> stateManager.setState(RobotState.SCORE_CORAL)));

        operator.povUp().onTrue(
                new InstantCommand(() -> stateManager.setState(RobotState.SCORE_ALGAE)));

        operator.povDown().onTrue(
                new InstantCommand(() -> stateManager.setState(RobotState.INTAKE_ALGAE)));

        operator.start().onTrue(
                new InstantCommand(() -> stateManager.setState(RobotState.CLIMB)));

        // === Vision Alignment ===
        driver.button(Constants.Joystick.STRAFE_RIGHT)
                .onTrue(new InstantCommand(() -> stateManager.setState(RobotState.ALIGN_VISION)));

        // === Cancel Align ===
        driver.button(Constants.Joystick.FUNCTION_1)
                .onTrue(new InstantCommand(() -> stateManager.setState(RobotState.DRIVE)));

        // === Manual Override ===
        operator.back().onTrue(
                new InstantCommand(() -> stateManager.setState(RobotState.MANUAL_OVERRIDE)));
    }


    // =======================================================================
    //                      DEFAULT COMMANDS (STATE AWARE)
    // =======================================================================

    private void configureDefaultCommands() {

        drivetrain.setDefaultCommand(
                Commands.run(() -> {

                    RobotState s = stateManager.getState();

                    if (s == RobotState.DRIVE || s == RobotState.ALIGN_VISION) {

                        double vx = -driver.getY() * MaxSpeed * Constants.MASTER_DRIVE_MULTIPLIER;
                        double vy = -driver.getX() * MaxSpeed * Constants.MASTER_DRIVE_MULTIPLIER;
                        double rot = -driver.getTwist() * MaxAngularRate * Constants.MASTER_DRIVE_MULTIPLIER;

                        if (s == RobotState.ALIGN_VISION) {
                            drivetrain.driveTeleop(vx, vy, rot, true, vision);
                        } else {
                            drivetrain.driveTeleop(vx, vy, rot, false, vision);
                        }

                    } else {
                        drivetrain.stop();
                    }
                }, drivetrain));

        // Effector manual override only works in MANUAL mode
        effector.setDefaultCommand(
                Commands.run(() -> {

                    if (stateManager.getState() != RobotState.MANUAL_OVERRIDE) {
                        effector.stop();
                        return;
                    }

                    double lt = operator.getLeftTriggerAxis();
                    double rt = operator.getRightTriggerAxis();

                    if (lt > 0.1) {
                        effector.start(-40 * lt);
                    } else if (rt > 0.1) {
                        effector.start(40 * rt);
                    } else {
                        effector.stop();
                    }

                }, effector));
    }


    // =======================================================================
    //                         PATHPLANNER NAMED COMMANDS
    // =======================================================================

    private void configureNamedCommands() {

        NamedCommands.registerCommand("INTAKE_CORAL",
                new InstantCommand(() -> stateManager.setState(RobotState.INTAKE_CORAL)));

        NamedCommands.registerCommand("SCORE_CORAL",
                new InstantCommand(() -> stateManager.setState(RobotState.SCORE_CORAL)));

        NamedCommands.registerCommand("ALIGN_VISION",
                new InstantCommand(() -> stateManager.setState(RobotState.ALIGN_VISION)));

        NamedCommands.registerCommand("INTAKE_ALGAE",
                new InstantCommand(() -> stateManager.setState(RobotState.INTAKE_ALGAE)));

        NamedCommands.registerCommand("SCORE_ALGAE",
                new InstantCommand(() -> stateManager.setState(RobotState.SCORE_ALGAE)));

        NamedCommands.registerCommand("CLIMB",
                new InstantCommand(() -> stateManager.setState(RobotState.CLIMB)));

        NamedCommands.registerCommand("IDLE",
                new InstantCommand(() -> stateManager.setState(RobotState.IDLE)));
    }


    // =======================================================================
    //                           AUTONOMOUS CHOICE
    // =======================================================================

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
