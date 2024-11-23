package frc.robot.subsystems.Auton;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoTracker {
    SequentialCommandGroup commands = new SequentialCommandGroup();
    PathPlannerPath intakingpath;
    PathPlannerPath Shootingpath;
    public AutoTracker(Boolean useVision, AutoSubsystems subsystems, List<AutoSector> paths){
        for (AutoSector autoSector : paths) {
            try {
                intakingpath = PathPlannerPath.fromPathFile(autoSector.intakingPath());
                Shootingpath = PathPlannerPath.fromPathFile(autoSector.ShootingPath());
                commands.addCommands(AutoBuilder.followPath(intakingpath));
            } catch (Exception e) {
            }
        }
    }
    public Command asCommand(){
        return commands;
    }
}
