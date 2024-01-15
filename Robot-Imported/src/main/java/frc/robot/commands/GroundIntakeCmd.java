package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Interfaces.IGroundIntake;

import com.revrobotics.CANSparkLowLevel.MotorType;

//import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;

public class GroundIntakeCmd extends Command {
    private IGroundIntake m_IntakeSubsystem;
    CANSparkMax NeoIntakeR = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax NeoIntakeL = new CANSparkMax(5, MotorType.kBrushless);
    XboxController js = new XboxController(0);

    final double SpeakerShooterSpeed = 1;
    final double AmpShooterSpeed = 0.3;
    final double IntakeShooterSpeed = -0.5;
    final double KickerSpeed = -1;
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public GroundIntakeCmd(IGroundIntake IntakeSubsystem) {
        this.m_IntakeSubsystem = IntakeSubsystem;
        addRequirements(m_IntakeSubsystem.asSubsystem());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // For the Intake
        if (js.getRawButton(1)) { // X
            //intake
            NeoIntakeR.set(IntakeShooterSpeed);
            NeoIntakeL.set(-IntakeShooterSpeed);
        }
        else if (js.getRawButton(2)){ // A
            //note zoom under bot
            NeoIntakeR.set(IntakeShooterSpeed);
            NeoIntakeL.set(IntakeShooterSpeed);
        }
        else {
            //stop
            NeoIntakeR.set(0);
            NeoIntakeL.set(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}