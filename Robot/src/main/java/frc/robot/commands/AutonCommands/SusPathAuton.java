package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class SusPathAuton {

  public SequentialCommandGroup getPath(AutonSubsystem m_AutonSubsystem, DriveSubsystem m_DriveSubsystem) {


    return new SequentialCommandGroup(
      m_AutonSubsystem.getTrajectoryCommand(m_DriveSubsystem, "suspath1", true, 3, 4), 
      new WaitCommand(5), 
      m_AutonSubsystem.getTrajectoryCommand(m_DriveSubsystem, "suspath2", true, 3, 4)
    
    ); 
  }
}
