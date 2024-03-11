package frc.robot.subsystems;

import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    DigitalOutput green;
    DigitalOutput red;
    DigitalOutput blue;
    DigitalOutput green1;
    DigitalOutput red1;
    DigitalOutput blue1;

    public void startup() {
        if (red == null) {
            red = new DigitalOutput(6);
            green = new DigitalOutput(4);
            blue = new DigitalOutput(2);
            red1 = new DigitalOutput(7);
            green1 = new DigitalOutput(5);
            blue1 = new DigitalOutput(3);
            setColor(Color.kBlack);
        }
        
    }


    public void setColor(Color m_color) {
        red.disablePWM();
        green.disablePWM();
        blue.disablePWM();

        red1.disablePWM();
        green1.disablePWM();
        blue1.disablePWM();

        red.enablePWM(m_color.red);
        green.enablePWM(m_color.green);
        blue.enablePWM(m_color.blue);

        red1.enablePWM(m_color.red);
        green1.enablePWM(m_color.green);
        blue1.enablePWM(m_color.blue);
    }

    public void turnLEDSOff() {
        setColor(Color.kBlack);
    }

    public void noteAlmostInBot(){
        setColor(Color.kOrangeRed);
    }

    public void noteInBot() {
        red.disablePWM();
        green.disablePWM();
        blue.disablePWM();

        red1.disablePWM();
        green1.disablePWM();
        blue1.disablePWM();

        red.enablePWM(0);
        green.enablePWM(191.25);
        blue.enablePWM(0);

        red1.enablePWM(0);
        green1.enablePWM(191.25);
        blue1.enablePWM(0);
    }
}