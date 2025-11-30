package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.state.*;
import frc.robot.subsystems.*;
import frc.robot.util.TagUtils.tagSide;

/**
 * Full state-machine RobotContainer
 */
public class RobotContainer {

    // State Machine
    private final RobotStateManager sm = new RobotStateManager();

    private final SendableChooser<Command> autoChooser;
    public static SendableChooser<Integer> positionChooser;

    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static double MaxAngularRate = RadiansPerSecond.of(3.0);

    // Drive request
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01)
            .withRotationalDeadband(MaxAngularRate * 0.01)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static final CommandJoystick driver = new CommandJoystick(0);
    public static final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator(sm);
    public final Effector effector = new Effector(sm);
    public final Intake intake = new Intake(sm);
    public final Hang hang = new Hang(sm);
    public final Lights lights = new Lights(sm);    
    public final Vision vision = new Vision();

    public RobotContainer() {

        configureBindings();
        configureDefaultCommands();

        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        positionChooser = new SendableChooser<>();
        positionChooser.setDefaultOption("Id 7", 1);
        positionChooser.addOption("Skip", 2);
        positionChooser.addOption("Id 8", 3);
        SmartDashboard.putData("Pose start", positionChooser);
    }

    /**
     * Button bindings converted to state transitions
     */
    private void configureBindings() {

        // ─────────────────────────────────────────────
        // DRIVER CONTROLS (Joystick 0)
        // ─────────────────────────────────────────────

        // Auto-align (full auto)
        driver.trigger().whileTrue(new InstantCommand(() -> sm.setState(RobotState.ALIGN_VISION)));

        // Cancel align or forced IDLE
        driver.button(2).onTrue(new InstantCommand(() -> sm.setState(RobotState.DRIVE)));

        // Strafe Left/Right using vision-assisted targeting
        driver.button(3).onTrue(new InstantCommand(() -> sm.setState(RobotState.ALIGN_VISION)));
        driver.button(4).onTrue(new InstantCommand(() -> sm.setState(RobotState.ALIGN_VISION)));

        // ─────────────────────────────────────────────
        // OPERATOR CONTROLS (Xbox Controller)
        // ─────────────────────────────────────────────

        // Intake coral
        operator.x().onTrue(new InstantCommand(() -> sm.setState(RobotState.INTAKE_CORAL)));

        // Score coral
        operator.a().onTrue(new InstantCommand(() -> sm.setState(RobotState.SCORE_CORAL)));

        // Algae up/down
        operator.povUp().onTrue(new InstantCommand(() -> sm.setState(RobotState.SCORE_ALGAE)));
        operator.povDown().onTrue(new InstantCommand(() -> sm.setState(RobotState.INTAKE_ALGAE)));

        // Climb
        operator.start().onTrue(new InstantCommand(() -> sm.setState(RobotState.CLIMB)));

        // Manual override
        operator.back().onTrue(new InstantCommand(
            () -> sm.setState(RobotState.MANUAL_OVERRIDE)
        ));
        operator.back().onFalse(new InstantCommand(
            () -> sm.setState(RobotState.IDLE)
        ));

        // Reset elevator to 0
        operator.y().onTrue(new InstantCommand(() -> elevator.toPosition(0)));
    }
    /**
     * Default commands for subsystems
     */
    private void configureDefaultCommands() {

        // Default driving when in DRIVE state
        drivetrain.setDefaultCommand(
            new RunCommand(() -> {
                if (sm.getState() == RobotState.DRIVE ||
                    sm.getState() == RobotState.IDLE ||
                    sm.getState() == RobotState.MANUAL_OVERRIDE) {

                    drivetrain.setControl(drive
                        .withVelocityX(driver.getY() * MaxSpeed)
                        .withVelocityY(driver.getX() * MaxSpeed)
                        .withRotationalRate(-driver.getTwist() * MaxAngularRate));
                }
            }, drivetrain)
        );

        // Manual override for effector (Triggers)
        effector.setDefaultCommand(
            new RunCommand(() -> {

                if (sm.getState() != RobotState.MANUAL_OVERRIDE) return;

                double lt = operator.getLeftTriggerAxis();
                double rt = operator.getRightTriggerAxis();

                if (lt > 0.1) effector.start(-70 * lt);
                else if (rt > 0.1) effector.start(70 * rt);
                else effector.stop();

            }, effector)
        );
    }

    /**
     * Auto-align Vision State logic (full auto align)
     */
    private void handleVisionAlign() {

        if (sm.getState() != RobotState.ALIGN_VISION) return;

        var tagPose = vision.getLatestPose();
        if (tagPose == null) {
            // No vision → fallback to DRIVE
            sm.setState(RobotState.DRIVE);
            return;
        }

        var current = drivetrain.getPose();
        var target = tagPose;

        // Offset adjustments (from your old MakeGoToTag)
        double desiredX = target.getX() - 0.30;   // forward offset
        double desiredY = target.getY();          // lateral offset

        double errorX = desiredX - current.getX();
        double errorY = desiredY - current.getY();
        double errorHeading = target.getRotation().minus(current.getRotation()).getRadians();

        boolean aligned =
            Math.abs(errorX) < 0.05 &&
            Math.abs(errorY) < 0.05 &&
            Math.abs(errorHeading) < Math.toRadians(3);

        if (aligned) {
            sm.setState(RobotState.DRIVE);
            return;
        }

        // Simple proportional approach
        double k = 1.5;

        drivetrain.setControl(
            drive
                .withVelocityX(k * errorX)
                .withVelocityY(k * errorY)
                .withRotationalRate(k * errorHeading)
        );
    }
    /**
     * Main periodic for RobotContainer-driven state tasks
     */
    public void periodic() {
        handleVisionAlign();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
