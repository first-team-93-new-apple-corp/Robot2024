package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.F310D;
import edu.wpi.first.wpilibj2.command.Command;


public class ShooterCommand extends Command{
    XboxController js = new XboxController(4);
    private ShooterCommand m_ShooterCommand;
    double SpeakerShooterSpeed = -0.6;
    double currentspeed;
    final double AmpShooterSpeed = 0.3;
    final double IntakeShooterSpeed = 0.75;
    final double KickerSpeed = -1;
    @Override
    public void execute() {
        ShooterSubsystem.shootConstants();
        IntakeSubsystem.IntakeConstants();
        // Stuff for the shooter
        if (js.getRawButton(F310D.RightTrigger)){ // RightTrigger
            ShooterSubsystem.shootSpeaker();
        }
        else if (js.getRawButton(F310D.RightShoulderButton)){ // RightShoulderButton
            ShooterSubsystem.shootAmp();
        }
        else if (js.getRawButton(F310D.LeftShoulderButton)){ // LeftShoulderButton
            ShooterSubsystem.shootMuzzle();
        }
        else {
            ShooterSubsystem.shootStop();
        }
        // For the Kicker
        if (js.getRawButton(F310D.B)) { // B
            ShooterSubsystem.shootIntake();
        }
        else {
            ShooterSubsystem.shootIntakeStop();
        }
        //Shooters motor speed control
        if (js.getRawButtonReleased(F310D.Start)) { //Start
            ShooterSubsystem.shootMinus();
        }
        if (js.getRawButtonReleased(F310D.Back)) {//Back
            ShooterSubsystem.shootPlus();
        }                          
    }
}

