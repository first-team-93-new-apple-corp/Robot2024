
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
import frc.robot.subsystems.DummyShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveState;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.Status;

public class DummyShooterCmd extends CommandBase{
    Joystick Dummyjoystick = new Joystick(0);
	public DummyShooterCmd() {
		 if (Dummyjoystick.getRawButtonPressed(4)){
            //Shoot Amp

        } else if (Dummyjoystick.getRawButton(5)){
            //Stop
            
        } else if (Dummyjoystick.getRawButtonPressed(6)) {
            //Shoot Speaker
            
        }
	}
}

