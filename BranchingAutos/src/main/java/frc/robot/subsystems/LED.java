package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;

public class LED implements Subsystem {
    //LED Definitions
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    // Patterns
    @SuppressWarnings("unused") 
    private LEDPattern m_RedBlueCycle;

    //Rainbow
    private static final Distance kLedSpacing = Feet.of(3 / 144.0);
    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 255);
    private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    // Funny Cycle
    public int lastI = 0;

    public LED() {
        // PWM port 9
        m_led = new AddressableLED(9);

        // Reuse buffer
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(144);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @SuppressWarnings("unused")
	private void applyColorCycle(int LedSpacing, Color Color1, Color Color2) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            int j = i;
            j += lastI;
            if (j % LedSpacing == 0) {
                m_ledBuffer.setLED(i, Color1);
            } else {
                m_ledBuffer.setLED(i, Color2);
            }
        }
        lastI++;
        if (lastI == 4) {
            lastI = 0;
        }
        m_led.setData(m_ledBuffer);
    }

    public void applyRainbow() {
        m_scrollingRainbow.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    public void closeLED() {
        m_led.stop();
    }
    public class LEDCommand implements Subsystem{
        public LEDCommand(){

        }
        public Command applyColorCycle(int LedSpacing, Color Color1, Color Color2) {
            return run(() -> applyColorCycle(LedSpacing, Color1, Color2));
        }
    }
}
