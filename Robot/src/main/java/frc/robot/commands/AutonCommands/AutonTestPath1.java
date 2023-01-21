package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoLevellingCommand;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.DriveSubsystem;


/**
 * Auton Path 1 Command
 * 
 */
public class AutonTestPath1 {

    /**
   * Generates the Auton Path 1 Command
   * @param m_AutonSubsystem Auton Subsystem
   * @param m_DriveSubsystem Drive Subsystem
   * @return Auton Path 1 Command
   * 
   */
  public static SequentialCommandGroup generatePath(AutonSubsystem m_AutonSubsystem, DriveSubsystem m_DriveSubsystem) {

    return new SequentialCommandGroup(
      m_AutonSubsystem.getTrajectoryCommand(m_DriveSubsystem, "Drive", true,1, .5),
      // new WaitCommand(2), 
      new AutoLevellingCommand(m_DriveSubsystem)
      // m_AutonSubsystem.getTrajectoryCommand(m_DriveSubsystem, "suspath2", true, 15, 5)
    
    ); 
  }
}
