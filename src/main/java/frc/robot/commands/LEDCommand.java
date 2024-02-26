package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommand extends Command {
    LEDSubsystem LEDSubsystem;
    Joystick op = new Joystick(1);

    public LEDCommand() {
        LEDSubsystem = new LEDSubsystem();
    }

    @Override
    public void execute() {
        if (op.getRawButton(Constants.Thrustmaster.Right_Buttons.Top_Left)) { //Change VERY SOON
            LEDSubsystem.noteInBot();
        } else if (op.getRawButton(Constants.Thrustmaster.Right_Buttons.Top_Middle)) { //CHANGER VERY SOON
            LEDSubsystem.turnLEDSOff();
        }
    }
    public void initialize() {
        LEDSubsystem.Startup();
    }
}