package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Lights extends SubsystemBase {
    private static final int m_Port = 1;
    private static final int m_Length = 45;
  
    public final AddressableLED m_LED;
    public final AddressableLEDBuffer m_buffer;
  
    public Lights() {
      m_LED = new AddressableLED(m_Port);
      m_buffer = new AddressableLEDBuffer(m_Length);
      m_LED.setLength(m_Length);
    }

    public void lightsOn(LEDPattern pattern) {
      pattern.applyTo(m_buffer);
      m_LED.setData(m_buffer);
      m_LED.start();
    }

  }