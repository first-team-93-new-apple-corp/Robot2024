package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;


public class ShooterAndIntakeCmd extends Command{
    // DCMotor Kicker
    TalonFX ShooterR = new TalonFX(05);
    TalonFX ShooterL = new TalonFX(11);
    CANSparkMax IntoShooter = new CANSparkMax(3, MotorType.kBrushless);
    CANSparkMax NeoIntakeR = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax NeoIntakeL = new CANSparkMax(5, MotorType.kBrushless);
    XboxController js = new XboxController(0);
    private ShooterAndIntakeCmd m_ShooterAndIntakeCmd;
    double SpeakerShooterSpeed = -0.6;
    double currentspeed;
    final double AmpShooterSpeed = 0.3;
    final double IntakeShooterSpeed = 0.75;
    final double KickerSpeed = -1;
    @Override
    public void execute() {
        SmartDashboard.putNumber("Shooter Speed", SpeakerShooterSpeed);

        // Stuff for the shooter
        if (js.getRawButton(8)){ // RT
            if (SpeakerShooterSpeed >= 0.6){
            ShooterR.set( SpeakerShooterSpeed);
            ShooterL.set( -SpeakerShooterSpeed);
        }
        }
        else if (js.getRawButton(6)){ // RB
            ShooterR.set( AmpShooterSpeed);
            ShooterL.set( -AmpShooterSpeed);
        }
        else if (js.getRawButton(5)){ // LB
            ShooterR.set( IntakeShooterSpeed);
            ShooterL.set( -IntakeShooterSpeed);
        }
        else {
            ShooterR.set(0);
            ShooterL.set(0);
        }
        // For the Kicker
        if (js.getRawButton(3)) { // B
            IntoShooter.set(KickerSpeed);
        }
        else {
            IntoShooter.set(0);
        }
        // For the Intake
        if (js.getRawButton(1)) { // X
            NeoIntakeR.set(IntakeShooterSpeed);
            NeoIntakeL.set(-IntakeShooterSpeed);
        }
        else if (js.getRawButton(2)){ // A
            NeoIntakeR.set(IntakeShooterSpeed);
            NeoIntakeL.set(IntakeShooterSpeed);
        }
        else {
            NeoIntakeR.set(0);
            NeoIntakeL.set(0);
        }
        //Shooters motor speed control
        if (js.getRawButtonReleased(9)) {
            SpeakerShooterSpeed -= 0.05;
            currentspeed = SpeakerShooterSpeed;
            SmartDashboard.putNumber("CurrentSpeed", currentspeed);
        }
        if (js.getRawButtonReleased(10)) {
            SpeakerShooterSpeed += 0.05;
            currentspeed = SpeakerShooterSpeed;
            SmartDashboard.putNumber("CurrentSpeed", currentspeed);
        }                          
    }
}

