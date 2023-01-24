package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.DriveSubsystem;


/**
 * Auton Path 1 Command
 * 
 */
public class CableBumpBlue1Pickup {

    /**
   * Generates the Auton Path 1 Command
   * @param m_AutonSubsystem Auton Subsystem
   * @param m_DriveSubsystem Drive Subsystem
   * @return Auton Path 1 Command
   * 
   */
  public static SequentialCommandGroup generatePath(AutonSubsystem m_AutonSubsystem, DriveSubsystem m_DriveSubsystem) {

    return new SequentialCommandGroup(
      m_AutonSubsystem.getTrajectoryCommand(m_DriveSubsystem, "CableBumpBlue1Pickup", true,3,1),
      new AutoStopDriveCommand(m_DriveSubsystem)

    
    ); 
  }
}
