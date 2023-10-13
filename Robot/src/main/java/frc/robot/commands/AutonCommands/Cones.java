package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class Cones {
    public static SequentialCommandGroup generatePath(AutonSubsystem m_AutonSubsystem,
            DriveSubsystem m_DriveSubsystem) {

        return new SequentialCommandGroup(
                m_AutonSubsystem.getTrajectoryCommand(m_DriveSubsystem, "Cones", true, 12, 12.5),
                new AutoStopDriveCommand(m_DriveSubsystem));
    }
}
