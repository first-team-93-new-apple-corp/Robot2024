package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
      m_AutonSubsystem.getTrajectoryCommand(m_DriveSubsystem, "suspath1", true, 3, 4), 
      new WaitCommand(5), 
      m_AutonSubsystem.getTrajectoryCommand(m_DriveSubsystem, "suspath2", true, 3, 4)
    
    ); 
  }
}
