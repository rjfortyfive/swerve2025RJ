package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.RobotState;
import frc.robot.state.RobotStateManager;

public class Lights extends SubsystemBase {
    private static final int kPort = 1;
    private static final int kLength = 45;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;

    private final RobotStateManager stateManager;
    private final Map<RobotState, Color> stateColors = new EnumMap<>(RobotState.class);

    public Lights(RobotStateManager stateManager) {
        this.stateManager = stateManager;

        m_led = new AddressableLED(kPort);
        m_buffer = new AddressableLEDBuffer(kLength);
        m_led.setLength(kLength);
        m_led.start();

        // Simple mapping from state → color
        stateColors.put(RobotState.IDLE, Color.kBlue);
        stateColors.put(RobotState.DRIVE, Color.kGreen);
        stateColors.put(RobotState.ALIGN_VISION, Color.kYellow);
        stateColors.put(RobotState.INTAKE_CORAL, Color.kOrange);
        stateColors.put(RobotState.SCORE_CORAL, Color.kRed);
        stateColors.put(RobotState.INTAKE_ALGAE, Color.kDarkGreen);
        stateColors.put(RobotState.SCORE_ALGAE, Color.kPurple);
        stateColors.put(RobotState.CLIMB, Color.kWhite);
        stateColors.put(RobotState.DISABLED, Color.kBlack);
    }

    @Override
    public void periodic() {
        RobotState s = stateManager.getState();
        Color c = stateColors.getOrDefault(s, Color.kBlue);

        for (int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setLED(i, c);
        }
        m_led.setData(m_buffer);
    }
}
