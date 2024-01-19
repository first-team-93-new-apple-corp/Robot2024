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

public class IntakeSubsystem extends SubsystemBase{
    static CANSparkMax NeoIntakeR = new CANSparkMax(4, MotorType.kBrushless); 
    static CANSparkMax NeoIntakeL = new CANSparkMax(5, MotorType.kBrushless);

    final static double IntakeShooterSpeed = 0.75;
    public static void Intake(){
        NeoIntakeR.set(IntakeShooterSpeed);
        NeoIntakeL.set(-IntakeShooterSpeed);
    }
    public static void IntakePassover(){
        NeoIntakeR.set(IntakeShooterSpeed);
        NeoIntakeL.set(-IntakeShooterSpeed);
    }
    public static void IntakeStop(){
        NeoIntakeR.set(IntakeShooterSpeed);
        NeoIntakeL.set(-IntakeShooterSpeed);
    }
    public static void IntakeConstants(){
        
    }
}
