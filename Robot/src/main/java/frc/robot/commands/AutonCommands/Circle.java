package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class Circle {
    public static SequentialCommandGroup generatePath(AutonSubsystem m_AutonSubsystem,
            DriveSubsystem m_DriveSubsystem) {

        return new SequentialCommandGroup(
                m_AutonSubsystem.getTrajectoryCommand(m_DriveSubsystem, "Circle", true, 10, 10),
                new AutoStopDriveCommand(m_DriveSubsystem));
    }
}
