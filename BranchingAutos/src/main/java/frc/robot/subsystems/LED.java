package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;

public class LED {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 60);
    private static final Distance kLedSpacing = Feet.of(3/144.0);
    private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
    private final LEDPattern Red = LEDPattern.solid(Color.kRed);
    private final LEDPattern Blue = LEDPattern.solid(Color.kBlue);

    public LED() {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
         m_led = new AddressableLED(9);
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(144);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();


    }
    public void applyRainbow(){
        m_scrollingRainbow.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }
    public void closeLED(){
        m_led.stop();
    }

}
