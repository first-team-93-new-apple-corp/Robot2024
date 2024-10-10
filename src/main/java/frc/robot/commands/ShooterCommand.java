package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class ShooterCommand extends Command {
    XboxController opController = new XboxController(2);
    Joystick driver2 = new Joystick(1);
    ShooterSubsystem m_ShooterSubsystem;
    LEDSubsystem m_LED;
    POVButton up = new POVButton(opController, 0);
    POVButton down = new POVButton(opController, 180);
    public ShooterCommand(ShooterSubsystem m_ShooterSubsystem, LEDSubsystem m_LED) {
        this.m_ShooterSubsystem = m_ShooterSubsystem;
        this.m_LED = m_LED;
    }

    @Override
    public void execute() {
        // Stuff for the shooter
        if (opController.getRawAxis(Constants.xbox.Axis.RT) > 0.6) { // RightTrigger
            m_LED.toggleVibeOff();
            m_ShooterSubsystem.prime();
            m_LED.LEDSHOOT().schedule();
        } else if (opController.getRawButton(Constants.xbox.RightShoulderButton)) { // RightShoulderButton
            m_ShooterSubsystem.shootAmp();
        } else if (opController.getRawButton(Constants.xbox.LeftShoulderButton)) { // LeftShoulderButton
            m_ShooterSubsystem.intakeFront();
        } else if (!opController.getRawButton(Constants.xbox.X)){
            m_ShooterSubsystem.shooterStop();
            // m_LED.toggleVibeOn();
        }

        if (opController.getRawButtonPressed(Constants.xbox.Menu)) {
            m_ShooterSubsystem.increaseSpeed();
        } else if (opController.getRawButtonPressed(Constants.xbox.Window)) {
            m_ShooterSubsystem.decreaseSpeed();
        }

        // For the Kicker
        if (
        driver2.getRawButton(Constants.Thrustmaster.Trigger)    
        // opController.getLeftTriggerAxis() > 0.01 
        // && !opController.getRawButton(Constants.xbox.RightShoulderButton)
        ) {
            m_ShooterSubsystem.kicker(1);
        } else if (driver2.getRawButton(Constants.Thrustmaster.Trigger) && opController.getRawButton(Constants.xbox.RightShoulderButton)) { 
            m_ShooterSubsystem.ampKicker();
        } else if (!opController.getRawButton(Constants.xbox.LeftShoulderButton) && !opController.getRawButton(Constants.xbox.X)) {
            m_ShooterSubsystem.kickerStop();
        }

        // For the LED

        // if (driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) <= 0 && driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) < .2) {
        //     m_LED.vibeSpeed1();
        // } else if (driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) <= .2 && driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) < .4) {
        //     m_LED.vibeSpeed2();
        // } else if (driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) <= .4 && driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) < .6) {
        //     m_LED.vibeSpeed3();
        // } else if (driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) <= .6 && driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) < .8) {
        //     m_LED.vibeSpeed4();
        // } else if (driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) <= .8 && driver2.getRawAxis(Constants.Thrustmaster.Axis.slider) < 1) {
        //     m_LED.vibeSpeed5();
        // } 

        // if (opController.getRawAxis(Constants.xbox.Axis.RT) > 0.6) { // RightTrigger
        //     m_LED.LEDSHOOT();
        // } 
        // else if (driver2.getRawButton(Constants.Thrustmaster.Right_Buttons.Top_Right)){
        //     m_LED.toggleVibeOn();
        // }  // else if (driver2.getRawButton(Constants.Thrustmaster.Right_Buttons.Top_Middle)){
        //     m_LED.vibing();
        // }else if (driver2.getRawButton(Constants.Thrustmaster.Right_Buttons.Bottom_Right)){
        //     m_LED.toggleVibeOff();
        // } \

        // For the Elevator

        // if (opController.getRawButtonPressed(Constants.xbox.RightShoulderButton)){
        //     m_LED.ServoUp();
        // } else if (opController.getRawButtonReleased(Constants.xbox.LeftShoulderButton)){
        //     m_LED.ServoDown();
        // }
       
    }
}
