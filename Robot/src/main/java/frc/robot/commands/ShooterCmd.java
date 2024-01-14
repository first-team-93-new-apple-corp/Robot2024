
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.DriveConstants;
// import frc.robot.CustomRotationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IShooter;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveState;
import frc.robot.subsystems.IntakeSubsystem;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.Status;

public class ShooterCmd extends CommandBase{
    private IShooter m_ShooterSubsystem;
    Joystick js;
	public ShooterCmd(IShooter ShooterSubsystem) {
        this.m_ShooterSubsystem = ShooterSubsystem;
        addRequirements(m_ShooterSubsystem.asSubsystem());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
       if (js.getRawButtonPressed(4)){
            //Shoot Amp
            m_ShooterSubsystem.shootAmp();
        } else if (js.getRawButton(5)){
            //Stop
            m_ShooterSubsystem.stop();
        } else if (js.getRawButtonPressed(6)) {
            //Shoot Speaker
            m_ShooterSubsystem.shootSpeaker();
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}


