package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    DigitalOutput green;
    DigitalOutput green1;

    public void Startup() {
        green = new DigitalOutput(2);
        green1 = new DigitalOutput(3);
        setColor(Color.kWhite);
    }

    public void setColor(Color m_color) {
        green.disablePWM();
        green1.disablePWM();

        green.enablePWM(m_color.green);
        green1.enablePWM(m_color.green);
    }

    public void turnLEDSOff() {
        setColor(Color.kBlack);
    }

    public void noteInBot() {;
        green.disablePWM();
        green1.disablePWM();

        green.enablePWM(191.25);
        green1.enablePWM(191.25);
    }
}