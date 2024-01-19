package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;


public class ShooterAndIntakeCmd extends Command{
    // DCMotor Kicker
    // TalonFX ShooterR = new TalonFX(05);
    // TalonFX ShooterL = new TalonFX(11);
    // CANSparkMax IntoShooter = new CANSparkMax(3, MotorType.kBrushless);
    // CANSparkMax NeoIntakeR = new CANSparkMax(4, MotorType.kBrushless);
    // CANSparkMax NeoIntakeL = new CANSparkMax(5, MotorType.kBrushless);
    XboxController js = new XboxController(4);
    private ShooterAndIntakeCmd m_ShooterAndIntakeCmd;
    @Override
    public void execute() {
        ShooterSubsystem.shootConstants();
        IntakeSubsystem.IntakeConstants();
        // Stuff for the shooter
        if (js.getRawButton(Constants.F310D.RightTrigger)){ // RightTrigger
            ShooterSubsystem.shootSpeaker();
        }
        else if (js.getRawButton(Constants.F310D.RightShoulderButton)){ // RightShoulderButton
            ShooterSubsystem.shootAmp();
        }
        else if (js.getRawButton(Constants.F310D.LeftShoulderButton)){ // LeftShoulderButton
            ShooterSubsystem.shootMuzzle();
        }
        else {
            ShooterSubsystem.shootStop();
        }
        // For the Kicker
        if (js.getRawButton(Constants.F310D.B)) { // B
            ShooterSubsystem.shootIntake();
        }
        else {
            ShooterSubsystem.shootIntakeStop();
        }
        // For the Intake
        if (js.getRawButton(Constants.F310D.X)) { // X
            IntakeSubsystem.Intake();
        }
        else if (js.getRawButton(Constants.F310D.A)){ // A
            IntakeSubsystem.IntakePassover();
        }
        else {
            IntakeSubsystem.IntakeStop();
        }
        //Shooters motor speed control
        if (js.getRawButtonReleased(Constants.F310D.Start)) { //Start
            ShooterSubsystem.shootMinus();
        }
        if (js.getRawButtonReleased(Constants.F310D.Back)) {//Back
            ShooterSubsystem.shootPlus();
        }                          
    }
}

