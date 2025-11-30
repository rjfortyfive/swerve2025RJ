package frc.robot.state;

public enum RobotState {
    // Neutral / meta
    DISABLED,
    IDLE,
    MANUAL_OVERRIDE,

    // Driving
    DRIVE,
    ALIGN_VISION,

    // Coral game piece
    INTAKE_CORAL,
    SCORE_CORAL,

    // Algae game piece
    INTAKE_ALGAE,
    SCORE_ALGAE,

    // Climbing
    CLIMB
}
