package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterAndIntakeCmd;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;

public class ShooterSubsystem  extends SubsystemBase{
    static TalonFX ShooterR = new TalonFX(05);
    static TalonFX ShooterL = new TalonFX(11);
    static CANSparkMax IntoShooter = new CANSparkMax(3, MotorType.kBrushless);
    static double SpeakerShooterSpeed = -0.6;
    static double currentspeed;
    final static double AmpShooterSpeed = 0.3;
    final static double KickerSpeed = -1;
    public static void shootSpeaker(){
        if (SpeakerShooterSpeed >= 0.6){
            currentspeed = SpeakerShooterSpeed;
            ShooterR.set( SpeakerShooterSpeed);
            ShooterL.set( -SpeakerShooterSpeed);
        }
    }
    public static void shootAmp(){
        ShooterR.set( AmpShooterSpeed);
        ShooterL.set( -AmpShooterSpeed);
    }
    public static void shootIntake(){
        IntoShooter.set(KickerSpeed);
    }
    public static void shootMuzzle(){
        currentspeed = SpeakerShooterSpeed;
        ShooterR.set( -SpeakerShooterSpeed);
        ShooterL.set( SpeakerShooterSpeed);
    }
    public static void shootStop(){
        ShooterR.set(0);
        ShooterL.set(0);
    }
    public static void shootIntakeStop(){
        IntoShooter.set(0);
    }
    public static void shootPlus(){
        if (SpeakerShooterSpeed < 1.0){ //100% speed
        SpeakerShooterSpeed += 0.05; //+5%speed
        currentspeed = SpeakerShooterSpeed;
        SmartDashboard.putNumber("CurrentSpeed", currentspeed);
        }
    }
    public static void shootMinus(){
        if (SpeakerShooterSpeed > 0.6){ //60% speed
        SpeakerShooterSpeed -= 0.05; //-5%speed
        currentspeed = SpeakerShooterSpeed;
        SmartDashboard.putNumber("CurrentSpeed", currentspeed);
        }
    }
    public static void shootConstants(){
        SmartDashboard.putNumber("Shooter Speed", currentspeed);
        SmartDashboard.putNumber("ShooterRmp", ShooterR.getVelocity().getValueAsDouble());
    }
}
