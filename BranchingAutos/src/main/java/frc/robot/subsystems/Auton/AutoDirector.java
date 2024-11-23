package frc.robot.subsystems.Auton;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoDirector {
  AutoSubsystems subsystems;
  SendableChooser<Auto> autoChooser = new SendableChooser<Auto>();
  List<Auto> Autos = new ArrayList<>();
    public AutoDirector( AutoSubsystems subsystems){
        this.subsystems = subsystems;
      AddAutos();
    }

  public record Auto(String name, Command command, Pose2d initPose) {}
  
  public Auto selection(){
    return autoChooser.getSelected();
  }

  public void AddAutos(){
    Autos.add(doNothing());
    Autos.add(shoot());
    for (Auto auto : Autos) {
      autoChooser.addOption(auto.name, auto);
    }
    SmartDashboard.putData("AutoChooser", autoChooser);
  }

  //------------------------------------------Autos------------------------------------------
  public Auto doNothing() {
    return new Auto("doNothing", new InstantCommand(), new Pose2d());
  }
  private Auto shoot() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("b", "a"));

    AutoTracker tracker = new AutoTracker(true, subsystems,paths);
    
    return new Auto("simpleShootAuto", tracker.asCommand(), null);
  }
}