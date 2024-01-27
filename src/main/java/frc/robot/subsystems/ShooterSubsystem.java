package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorStates;
import edu.wpi.first.math.controller.PIDController;

public class ShooterSubsystem extends SubsystemBase {
    static TalonFX ShooterR = new TalonFX(16);
    static TalonFX ShooterL = new TalonFX(15);
    static CANSparkMax IntoShooter = new CANSparkMax(1, MotorType.kBrushless);
    static double SpeakerShooterSpeed = 0.45;
    static double currentspeed;
    static double MuzzleIntake = 0.25;
    final static double AmpShooterSpeed = 0.3;
    final static double KickerSpeed = -1;
    // Elevator
    static TalonFX elevatorMotor = new TalonFX(11);
    static final double stowSetpoint = 0;
    static final double ampSetpoint = 1000;
    static final double sourceSetpoint = 300;
    static final double trapSetpoint = 500;
    static final double speakerSetpoint = 700;
    ElevatorFeedforward elefeedforward = new ElevatorFeedforward(0,0,0,0); // change values
    PIDController elevatorPID = new PIDController(0,0,0);
    public ShooterSubsystem() {
        ShooterL.setInverted(false);
        ShooterR.setInverted(false);
    }
    public static void prime() {
        System.out.println("shooting speaker");
        Constants.elevatorState = ElevatorStates.SPEAKER;
        ShooterR.set(SpeakerShooterSpeed);
        ShooterL.set(-SpeakerShooterSpeed);

    }

    public static void shootAmp() {
        Constants.elevatorState = ElevatorStates.AMP;
        ShooterR.set(-AmpShooterSpeed);
        ShooterL.set(AmpShooterSpeed);
    }

    public static void kicker() {
        IntoShooter.set(KickerSpeed);
    }

    public static void shootMuzzle() {
        ShooterR.set(-MuzzleIntake);
        ShooterL.set(MuzzleIntake);
    }

    public static void shootStop() {
        ShooterR.set(0);
        ShooterL.set(0);
    }

    public static void shootIntakeStop() {
        IntoShooter.set(0);
    }

    public static void shootPlus() {
        if (SpeakerShooterSpeed < 0) {
            SpeakerShooterSpeed += 0.05; // +5%speed
        }
        SmartDashboard.putNumber("CurrentSpeed", SpeakerShooterSpeed);

    }

    public static void shootMinus() {
        if (SpeakerShooterSpeed > -0.95) {
            SpeakerShooterSpeed -= 0.05; // -5%speed
        }
        SmartDashboard.putNumber("CurrentSpeed", SpeakerShooterSpeed);

    }

    public static void shootConstants() {
        SmartDashboard.putNumber("Shooter Speed", SpeakerShooterSpeed);
        SmartDashboard.putNumber("ShooterRmp", ShooterR.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 1 Temp", ShooterL.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 2 Temp", ShooterR.getDeviceTemp().getValueAsDouble());
    }

    @Override
    public void periodic() {
        if (Constants.elevatorState == Constants.ElevatorStates.STOW) {
            elevatorMotor.set(elefeedforward.calculate(elevatorPID.calculate(elevatorMotor.getPosition().getValueAsDouble(), stowSetpoint)));

        } else if (Constants.elevatorState == Constants.ElevatorStates.AMP) {
            elevatorMotor.set(elefeedforward.calculate(elevatorPID.calculate(elevatorMotor.getPosition().getValueAsDouble(), ampSetpoint)));

        } else if (Constants.elevatorState == Constants.ElevatorStates.SOURCE) {
            elevatorMotor.set(elefeedforward.calculate(elevatorPID.calculate(elevatorMotor.getPosition().getValueAsDouble(), sourceSetpoint)));

        }else if (Constants.elevatorState == Constants.ElevatorStates.TRAP) {
            elevatorMotor.set(elefeedforward.calculate(elevatorPID.calculate(elevatorMotor.getPosition().getValueAsDouble(), trapSetpoint)));
            
        }else if (Constants.elevatorState == Constants.ElevatorStates.SPEAKER) {
            elevatorMotor.set(elefeedforward.calculate(elevatorPID.calculate(elevatorMotor.getPosition().getValueAsDouble(), speakerSetpoint)));
            
        }
    }
}
