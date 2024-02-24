package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    DigitalOutput red;
    DigitalOutput green;
    DigitalOutput blue;
    DigitalOutput red1;
    DigitalOutput green1;
    DigitalOutput blue1;

    int vibingCounter = 0;
    int loopsBetweenVibing = 1;
    int loopsCounter = 0;

    double r;
    double g;
    double b;

    public void Startup() {
        red = new DigitalOutput(6);
        green = new DigitalOutput(7);
        blue = new DigitalOutput(8);
        red1 = new DigitalOutput(5);
        green1 = new DigitalOutput(3);
        blue1 = new DigitalOutput(4);
        setColor(Color.kWhite);
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

    public void X(){
        setColor(Color.kBlue);
    }
    public void Y(){
        setColor(Color.kYellow);
    }
    public void A(){
        setColor(Color.kGreen);
    }
    public void B(){
        setColor(Color.kRed);
    }

    public void noteInBot(){
        red.disablePWM();
        green.disablePWM();
        blue.disablePWM();

        red1.disablePWM();
        green1.disablePWM();
        blue1.disablePWM();

        red.enablePWM(0);
        green.enablePWM(153);
        blue.enablePWM(0);

        red1.enablePWM(0);
        green1.enablePWM(191.25);
        blue1.enablePWM(0);
    }
}