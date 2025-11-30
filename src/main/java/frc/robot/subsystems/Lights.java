package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.state.*;

public class Lights extends SubsystemBase {
    private static final int kPort = 1;
    private static final int kLength = 45;
  
    public final AddressableLED m_led;
    public final AddressableLEDBuffer m_buffer;

    public static Color purple = Color.fromHSV(135, 220, 170);
    public static Color gold   = Color.fromHSV(23, 242, 214);
    public static LEDPattern purpleGoldStep = LEDPattern.steps(Map.of(0.0, purple, 0.5, gold));

    private final RobotStateManager sm;
  
    public Lights(RobotStateManager sm) {
        this.sm = sm;

        m_led = new AddressableLED(kPort);
        m_buffer = new AddressableLEDBuffer(kLength);
        m_led.setLength(kLength);
        m_led.start();
    }

    @Override
    public void periodic() {
        RobotState s = sm.getState();
        Color c;

        switch (s) {
            case DRIVE:          c = Color.kBlue;   break;
            case ALIGN_VISION:   c = Color.kYellow; break;
            case INTAKE_CORAL:   c = Color.kOrange; break;
            case SCORE_CORAL:    c = Color.kRed;    break;
            case INTAKE_ALGAE:   c = Color.kGreen;  break;
            case SCORE_ALGAE:    c = purple;        break;
            case CLIMB:          c = Color.kWhite;  break;
            case MANUAL_OVERRIDE:c = gold;          break;
            case DISABLED:       c = Color.kBlack;  break;
            default:             c = Color.kBlack;  break;
        }

        for (int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setLED(i, c);
        }
        m_led.setData(m_buffer);
    }

    public void lightsOn(LEDPattern pattern) {
      pattern.applyTo(m_buffer);
      m_led.setData(m_buffer);
      m_led.start();
    }
}
