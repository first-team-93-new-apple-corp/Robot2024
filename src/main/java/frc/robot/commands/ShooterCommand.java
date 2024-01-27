package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;


public class ShooterCommand extends Command{
    XboxController opController = new XboxController(2);
    Joystick m_Joystick2 = new Joystick(1);
    @Override
    public void execute() {
        ShooterSubsystem.shootConstants();
        // Stuff for the shooter
        if (opController.getRawButton(Constants.F310_D.RightTrigger)){ // RightTrigger
            ShooterSubsystem.prime();
        }
        else if (opController.getRawButton(Constants.F310_D.RightShoulderButton)){ // RightShoulderButton
            ShooterSubsystem.shootAmp();
        }
        else if (opController.getRawButton(Constants.F310_D.LeftShoulderButton)){ // LeftShoulderButton
            ShooterSubsystem.shootMuzzle();
        }
        else {
            ShooterSubsystem.shootStop();
        }
        // For the Kicker
        if (m_Joystick2.getRawButton(Constants.Thrustmaster.Trigger)) { // B
            ShooterSubsystem.kicker();
        }
        else {
            ShooterSubsystem.shootIntakeStop();
        }
        // For the Intake
        if (opController.getRawButton(Constants.F310_D.X)) { // X
            IntakeSubsystem.Intake();
        }
        else if (opController.getRawButton(Constants.F310_D.A)){ // A
            IntakeSubsystem.IntakePassover();
        }
        else {
            IntakeSubsystem.IntakeStop();
        }
        //Shooters motor speed control
        if (opController.getRawButtonReleased(Constants.F310_D.Start)) { //Start
            
            ShooterSubsystem.shootMinus();
        }
        if (opController.getRawButtonReleased(Constants.F310_D.Back)) {//Back
            ShooterSubsystem.shootPlus();
        }                          
    }
}

