package frc.robot.subsystems.Auton;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.TunerConstants;

public class AutoTracker {
    SequentialCommandGroup commands = new SequentialCommandGroup();
    PathPlannerPath intakingpath;
    PathPlannerPath Shootingpath;
    PathConstraints constraints = new PathConstraints(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), 15, 5, 10);
    public AutoTracker( AutoSubsystems subsystems, List<AutoSector> paths, Supplier<Pose2d> initalPose){
        commands.addCommands(Commands.runOnce(() -> subsystems.driveSubsystem().resetPose(initalPose.get()), subsystems.driveSubsystem()));
        for (AutoSector autoSector : paths) {
            try {
                intakingpath = PathPlannerPath.fromPathFile(autoSector.intakingPath());
                Shootingpath = PathPlannerPath.fromPathFile(autoSector.ShootingPath());
                commands.addCommands(AutoBuilder.followPath(intakingpath));
                Commands.print("Vision Note Grab");
                // commands.addCommands(Commands.runOnce(() -> subsystems.driveSubsystem()), subsystems.driveSubsystem()));
                commands.addCommands(AutoBuilder.pathfindThenFollowPath(Shootingpath, constraints));
                Commands.print("Bang Bang (shot the note)");
            } catch (Exception e) {}
        }
        try {
            // commands.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("Leave")));
        } catch (Exception e) {}
    }
    public Command asCommand(){
        return commands;
    }
}
