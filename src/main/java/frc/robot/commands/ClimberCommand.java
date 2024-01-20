package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.F310_D;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimberCommand extends Command{
    XboxController js = new XboxController(4);
   
    ClimberSubsystem ClimberSubsystem = new ClimberSubsystem();
    @Override
    public void execute() {
        
        if (js.getRawButtonReleased(F310_D.Y)) {
            ClimberSubsystem.raiseClimber();
        }
        if (js.getRawButtonReleased(F310_D.B)) {
            ClimberSubsystem.lowerClimber();
        }
    }
}