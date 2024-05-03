package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommand extends Command {
    LEDSubsystem m_LEDSubsystem;
    XboxController op;

    public LEDCommand(XboxController op, LEDSubsystem m_LEDSubsystem) {
        this.op = op;
        this.m_LEDSubsystem = m_LEDSubsystem;
    }

    @Override
    public void execute() {
        if (op.getRawButton(Constants.xbox.LeftPaddle)) {
            m_LEDSubsystem.noteInBot();
        } else {
            return;
        }
    }
}