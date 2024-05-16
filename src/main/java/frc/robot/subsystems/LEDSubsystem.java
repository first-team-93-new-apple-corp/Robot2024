package frc.robot.subsystems;

import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    DigitalOutput green;
    DigitalOutput red;
    DigitalOutput blue;
    DigitalOutput green1;
    DigitalOutput red1;
    DigitalOutput blue1;
    DigitalOutput servo;

    // Servo ampServo;

    boolean vibe;

    int loopsCounter;
    int loopsBetweenVibing = 1;
    int vibingCounter;
    int vibeSpeed;
    int shotCounter;

    XboxController op = new XboxController(2);
    Joystick driver2 = new Joystick(1);

    public void startup() {
        if (red == null) {
            red = new DigitalOutput(6);
            green = new DigitalOutput(4);
            blue = new DigitalOutput(2);
            red1 = new DigitalOutput(7);
            green1 = new DigitalOutput(5);
            blue1 = new DigitalOutput(3);
            // ampServo = new Servo(8);
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

    public void shot() {
        setColor(Color.kGray);
    }

    public Command LEDSHOOT() {
        return LEDOn(Color.kBlue).alongWith(Commands.waitSeconds(.5))
                .andThen(LEDNoMoreOn().alongWith(Commands.waitSeconds(.1)))
                .andThen(LEDOn(Color.kBlue).alongWith(Commands.waitSeconds(.3)))
                .andThen(LEDNoMoreOn().alongWith(Commands.waitSeconds(.1)))
                .andThen(LEDOn(Color.kBlue).alongWith(Commands.waitSeconds(.3))).andThen(LEDNoMoreOn());
    }

    public Command LEDDEMO() {
        return LEDOn(Color.kRed).alongWith(Commands.waitSeconds(1))
                .andThen(LEDOn(Color.kWhite).alongWith(Commands.waitSeconds(1)))
                .andThen(LEDOn(Color.kBlue).alongWith(Commands.waitSeconds(1)))
                .andThen(LEDNoMoreOn());
    }

    public Command LEDNoMoreOn() {
        return runOnce(() -> turnLEDSOff());
    }

    public Command LEDOn(Color m_Color) {
        return runOnce(() -> setColor(m_Color));
    }

    public void toggleVibeOff() {
        vibe = false;
        LEDNoMoreOn().schedule();
    }

    public void toggleVibeOn() {
        vibe = true;
    }

    public void vibing() {
        if (driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) <= 0
                && driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) < .2) {
            vibeSpeed1();
        } else if (driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) <= .2
                && driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) < .4) {
            vibeSpeed2();
        } else if (driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) <= .4
                && driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) < .6) {
            vibeSpeed3();
        } else if (driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) <= .6
                && driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) < .8) {
            vibeSpeed4();
        } else if (driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) <= .8
                && driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) < 1) {
            vibeSpeed5();
        }
        if (driver2.getRawButton(Constants.Thrustmaster.Right_Buttons.Top_Right)) {
            toggleVibeOn();
        } else if (driver2.getRawButton(Constants.Thrustmaster.Right_Buttons.Bottom_Right)) {
            toggleVibeOff();
        }

        if (op.getRawAxis(Constants.xbox.Axis.LT) > 0.6) { // LeftTrigger
            LEDDEMO();
        }

        if (vibe) {
            if (loopsCounter >= loopsBetweenVibing) {
                loopsCounter = 0;
                vibingCounter += vibeSpeed;
            } else {
                loopsCounter += vibeSpeed;
            }

            if (vibingCounter > 255) {
                vibingCounter = 0;

            }

            setColor(Color.fromHSV(vibingCounter, 255, 255));
        } else {
            if (op.getRawAxis(Constants.xbox.Axis.LT) > 0.6) { // LeftTrigger
                LEDDEMO().schedule();
            }
            return;
        }
    }

    public void vibeSpeed1() {
        vibeSpeed = 1;
    }

    public void vibeSpeed2() {
        vibeSpeed = 2;
    }

    public void vibeSpeed3() {
        vibeSpeed = 3;
    }

    public void vibeSpeed4() {
        vibeSpeed = 4;
    }

    public void vibeSpeed5() {
        vibeSpeed = 5;
    }

    public void noteAlmostInBot() {
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