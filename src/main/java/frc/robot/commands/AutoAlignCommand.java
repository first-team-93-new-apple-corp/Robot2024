
package frc.robot.commands;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

public class AutoAlignCommand extends Command {
    Joystick m_joystick1;


    SwerveDriveSubsystem m_DriveSubsystem;


    public AutoAlignCommand(SwerveDriveSubsystem drivetrain, Joystick stick1) {
        m_DriveSubsystem = drivetrain;
        m_joystick1= stick1;
    }


    @Override
    public void execute() {
        if (m_joystick1.getRawButton(Constants.Thrustmaster.Center_Button)) {
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                    m_DriveSubsystem.toPose(Constants.AprilTagPoseConstants.RedAmp);
                } else {
                    m_DriveSubsystem.toPose(Constants.AprilTagPoseConstants.BlueAmp);
                }
        }
        } else if(m_joystick1.getRawButton(Constants.Thrustmaster.Right_Button)){
            // m_AutoAlignSubsystem.AutoAimTrap();
        }
    }
}
