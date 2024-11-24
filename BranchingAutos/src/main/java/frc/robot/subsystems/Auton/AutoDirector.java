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

  public AutoDirector(AutoSubsystems subsystems) {
    this.subsystems = subsystems;
    AddAutos();
  }

  public record Auto(String name, Command command, Pose2d initPose) {
  }

  public Auto selection() {
      return autoChooser.getSelected();
  }

  public void AddAutos() {
    autoChooser.setDefaultOption(doNothing().name, doNothing());
    Autos.add(Speaker3());
    for (Auto auto : Autos) {
      autoChooser.addOption(auto.name, auto);
    }
    SmartDashboard.putData("AutoChooser", autoChooser);
  }

  // ------------------------------------------Autos------------------------------------------
  public Auto doNothing() {
    return new Auto("doNothing", new InstantCommand(), new Pose2d());
  }

  public Auto Speaker3() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("SG1", "SC"));
    paths.add(new AutoSector("SG2", "SC"));
    paths.add(new AutoSector("SG3", "SC"));
    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.Speaker());

    return new Auto("Speaker3", tracker.asCommand(), PositionConstants.Speaker() );
  }
  public Auto SG3() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("SG1", "SC"));
    paths.add(new AutoSector("SG2", "SC"));
    paths.add(new AutoSector("SG3", "SC"));
    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.Speaker());

    return new Auto("Speaker3", tracker.asCommand(), PositionConstants.Speaker() );
  }

}