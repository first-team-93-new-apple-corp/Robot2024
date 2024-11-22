package frc.robot.subsystems.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AutoDirector {
    Subsystem[] subsystems;
    AutoDirector( Subsystem[] subsystems){
        this.subsystems = subsystems;

    }

  public record Auto(String name, Command command, Pose2d initPose) {}


  //------------------------------------------Autos------------------------------------------
  public Auto doNothing() {
    return new Auto("doNothing", new InstantCommand(), new Pose2d());
  }
  private Auto shoot() {
    AutoTracker paths = new AutoTracker();
    
    return new Auto("simpleShootAuto", paths.asCommand(), null);
  }
}