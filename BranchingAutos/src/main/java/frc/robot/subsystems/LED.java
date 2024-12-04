package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import com.ctre.phoenix6.signals.Led1OffColorValue;

import edu.wpi.first.units.measure.Distance;

public class LED {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 255);
    private final LEDPattern m_bluetest;
    private static final Distance kLedSpacing = Feet.of(3 / 144.0);
    private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
    private final LEDPattern Red = LEDPattern.solid(Color.kRed);
    private final LEDPattern Blue = LEDPattern.solid(Color.kBlue);
    // private final LEDPattern Shoot = LEDPattern.steps();

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
        m_bluetest = LED.blueStrips(255,255);
    }
    
    public static LEDPattern blueStrips(int saturation, int value) {
        
        return (reader, writer) -> {
            int bufLen = reader.getLength();
            for (int i = 0; i < bufLen; i++) {
                // Create a pattern of blue and black strips
                int hue = 240;  // Blue hue in HSV (240 represents blue)
                if (i % 2 == 0) {
                    // Set full blue when i is even
                    writer.setHSV(i, hue, saturation, value);
                } else {
                    // Set black when i is odd (hue, saturation, and value all 0)
                    writer.setHSV(i, hue, 0, 0);
                }
            }
        };
    }
    
public void blueStripesRunning() {
    m_bluetest.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
}


    public void applyRainbow() {
        m_scrollingRainbow.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    public void redBlue() {
        Red.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
        Blue.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);

    }

    public void closeLED() {
        m_led.stop();
    }

}
