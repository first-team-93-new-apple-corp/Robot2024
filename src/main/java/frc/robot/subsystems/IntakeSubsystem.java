package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorStates;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class IntakeSubsystem extends SubsystemBase {
    static CANSparkMax NeoIntakeFront = new CANSparkMax(2, MotorType.kBrushless);
    static CANSparkMax NeoIntakeBack = new CANSparkMax(3, MotorType.kBrushless);
    final static boolean ifNote = false;
    final static double IntakeShooterSpeed = 0.75;

    public static void Intake() {
        // if (ifNote == false){
        Constants.elevatorState = ElevatorStates.STOW;
        NeoIntakeFront.set(IntakeShooterSpeed);
        NeoIntakeBack.set(-IntakeShooterSpeed);
        // }
    }

    public static void IntakePassover() { 
        // if (ifNote == true){
        NeoIntakeFront.set(IntakeShooterSpeed);
        NeoIntakeBack.set(-IntakeShooterSpeed);
        // }
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
