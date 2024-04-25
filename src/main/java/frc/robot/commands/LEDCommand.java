package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommand extends Command {
    LEDSubsystem LEDSubsystem;
    XboxController op;

    public LEDCommand(XboxController op) {
        this.op = op;
        LEDSubsystem = new LEDSubsystem();
    }

    public Command LEDShotCommand() {
        return LEDSubsystem.runOnce(() -> LEDSubsystem.Shot()).alongWith(Commands.waitSeconds(0.5)).andThen(() -> LEDSubsystem.turnLEDSOff());
    }
    @Override
    public void execute() {
        if (op.getRawButton(Constants.F310_D.LeftTrigger)) {
            LEDSubsystem.noteInBot();
        } else if (op.getRawButton(Constants.F310_D.RightTrigger)) {
            LEDSubsystem.turnLEDSOff();
        } else if (op.getRawButton(Constants.F310_D.Y)) {
            LEDSubsystem.Shot();
        } 
    }

    @Override
    public void initialize() {
        // LEDSubsystem.Startup();
    }
}
