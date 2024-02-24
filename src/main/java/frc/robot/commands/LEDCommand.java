package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommand extends Command {
    LEDSubsystem LEDSubsystem;
    XboxController op;

    public LEDCommand(XboxController op) {
        this.op = op;
        LEDSubsystem = new LEDSubsystem();
    }

    @Override
    public void execute() {
        if (op.getRawButton(Constants.F310_D.LeftTrigger)) {
            LEDSubsystem.noteInBot();
        } else if (op.getRawButton(Constants.F310_D.RightTrigger)) {
            LEDSubsystem.turnLEDSOff();
        } else if (op.getRawButton(Constants.F310_D.Y)) {
            LEDSubsystem.Y();
        } else if (op.getRawButton(Constants.F310_D.X)) {
            LEDSubsystem.X();
        } else if (op.getRawButton(Constants.F310_D.A)) {
            LEDSubsystem.A();
        } else if (op.getRawButton(Constants.F310_D.B)) {
            LEDSubsystem.B();
        }
    }

    @Override
    public void initialize() {
        // LEDSubsystem.Startup();
    }
}
