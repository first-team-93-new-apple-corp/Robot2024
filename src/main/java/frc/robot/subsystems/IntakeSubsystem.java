package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class IntakeSubsystem extends SubsystemBase {
    static CANSparkMax NeoIntakeR = new CANSparkMax(2, MotorType.kBrushless);
    static CANSparkMax NeoIntakeL = new CANSparkMax(3, MotorType.kBrushless);
    final static boolean ifNote = false;
    final static double IntakeShooterSpeed = 0.75;

    public static void Intake() {
        if (ifNote == false){
        NeoIntakeR.set(IntakeShooterSpeed);
        NeoIntakeL.set(-IntakeShooterSpeed);
        }
    }

    public static void IntakePassover() { 
        if (ifNote == true){
        NeoIntakeR.set(IntakeShooterSpeed);
        NeoIntakeL.set(-IntakeShooterSpeed);
        }
    }

    public static void IntakeStop() {
        NeoIntakeFront.set(0);
        NeoIntakeBack.set(0);
    }

    public static void IntakeConstants() {
        // if note is in kicker make ifNote True
        SmartDashboard.putBoolean("Note intaked", ifNote);
    }
}
