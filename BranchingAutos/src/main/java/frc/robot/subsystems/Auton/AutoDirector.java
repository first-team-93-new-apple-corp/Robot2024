package frc.robot.subsystems.Auton;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoDirector {
  AutoSubsystems subsystems;
    public AutoDirector( AutoSubsystems subsystems){
        this.subsystems = subsystems;

    }

  public record Auto(String name, Command command, Pose2d initPose) {}


  //------------------------------------------Autos------------------------------------------
  public Auto doNothing() {
    return new Auto("doNothing", new InstantCommand(), new Pose2d());
  }
  private Auto shoot() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("null", "null"));

    AutoTracker tracker = new AutoTracker(true, subsystems,paths);
    
    return new Auto("simpleShootAuto", tracker.asCommand(), null);
  }
}