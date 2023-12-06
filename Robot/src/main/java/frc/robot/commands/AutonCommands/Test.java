package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class Test {
    /**
     * Generates the Auton Path 1 Command
     * 
     * @param m_AutonSubsystem Auton Subsystem
     * @param m_DriveSubsystem Drive Subsystem
     * @return Auton Path 1 Command
     * 
     */
    public static SequentialCommandGroup generatePath(AutonSubsystem m_AutonSubsystem,
            DriveSubsystem m_DriveSubsystem) {

        return new SequentialCommandGroup(
                m_AutonSubsystem.getTrajectoryCommand(m_DriveSubsystem, "New Path", true, 4, 3),
                new LockWheels(m_DriveSubsystem));
    }
}
// Testing