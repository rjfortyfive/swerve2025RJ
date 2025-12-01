package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Lights extends SubsystemBase {
    private static final int kPort = 1;
    private static final int kLength = 45;
  
    public final AddressableLED m_led;
    public final AddressableLEDBuffer m_buffer;
  
    public Lights() {
      m_led = new AddressableLED(kPort);
      m_buffer = new AddressableLEDBuffer(kLength);
      m_led.setLength(kLength);
      
  
      // Set the default command to turn the strip off, otherwise the last colors written by
      // the last command to run will continue to be displayed.
      // Note: Other default patterns could be used instead!
      // setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));

      
    }

    public void lightsOn(LEDPattern pattern) {
      pattern.applyTo(m_buffer);
      m_led.setData(m_buffer);
      m_led.start();
    }

  }