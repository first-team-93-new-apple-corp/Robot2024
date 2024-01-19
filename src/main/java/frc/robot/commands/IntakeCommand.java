package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.F310D;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command{
    XboxController js = new XboxController(4);
    private IntakeCommand m_IntakeCommand;
    public void execute(){
    if (js.getRawButton(F310D.X)) { // X
        IntakeSubsystem.Intake();
    }
    else if (js.getRawButton(F310D.A)){ // A
        IntakeSubsystem.IntakePassover();
    }
    else {
        IntakeSubsystem.IntakeStop();
    }
}
}
