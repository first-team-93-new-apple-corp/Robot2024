package frc.robot.subsystems.Auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AutoTracker {
    SequentialCommandGroup commands = new SequentialCommandGroup();
    public void add(Command command, Subsystem subsystem){
        commands.addCommands(command);
        commands.addRequirements(subsystem);
    }
    public Command asCommand(){
        return null;
    }
}
