package frc.robot.subsystems.Auton;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoTracker {
    SequentialCommandGroup commands = new SequentialCommandGroup();
    public AutoTracker(Boolean useVision, AutoSubsystems subsystems, List<AutoSector> paths){
        for (AutoSector autoSector : paths) {
            
        }
    }
    public void add(Command command){
        commands.addCommands(command);
    }
    public Command asCommand(){
        return null;
    }
}
