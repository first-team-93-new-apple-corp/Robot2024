package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class IntakeSubsystem extends SubsystemBase {
    // static CANSparkMax NeoIntakeFront = new CANSparkMax(2, MotorType.kBrushless);
    // static CANSparkMax NeoIntakeBack = new CANSparkMax(3, MotorType.kBrushless);

    final static double IntakeShooterSpeed = 0.75;

    public static void Intake() {
        // NeoIntakeFront.set(IntakeShooterSpeed);
        // NeoIntakeBack.set(-IntakeShooterSpeed);
    }

    public static void IntakePassover() {
        // NeoIntakeFront.set(IntakeShooterSpeed);
        // NeoIntakeBack.set(-IntakeShooterSpeed);
    }

    public static void IntakeStop() {
        // NeoIntakeFront.set(0);
        // NeoIntakeBack.set(0);
    }

    public static void IntakeConstants() {

    }
}
