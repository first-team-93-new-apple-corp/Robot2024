package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Interfaces.IShooter;

public class ShooterSubsystem extends SubsystemBase implements IShooter{
    //Initations here later
    double CurrentSetpoint;
    final double AmpSetPoint = 65;
    final double IntakeSetPoint = -1;
    final double SpeakerSetpoint = -0.5;
    TalonFX ShooterL= new TalonFX(11);
    TalonFX ShooterR= new TalonFX(05);
    CANSparkMax NEOIntake = new CANSparkMax(3, MotorType.kBrushless);
    XboxController xjs = new XboxController(0);
    @Override
    public void intake() {
        CurrentSetpoint = IntakeSetPoint;
        System.out.println("Muzzle loading");
        NEOIntake.set(IntakeSetPoint);
    }
    @Override
    public void stop() {
        System.out.println("Stop shooting motors");
        ShooterL.setNeutralMode(NeutralMode.Brake);
        ShooterR.setNeutralMode(NeutralMode.Brake);
        NEOIntake.setIdleMode(IdleMode.kBrake);
        }
    @Override
    public void shootAmp() {
        CurrentSetpoint = AmpSetPoint;
        System.out.println("shootAmp at lower speed");
        ShooterL.set(TalonFXControlMode.PercentOutput, -AmpSetPoint);
        ShooterR.set(TalonFXControlMode.PercentOutput, AmpSetPoint);
    }
    @Override
    public void shootSpeaker() {
        CurrentSetpoint = SpeakerSetpoint;
        System.out.println("shootSpeaker at high speed");
        ShooterL.set(TalonFXControlMode.PercentOutput, -SpeakerSetpoint);
        ShooterR.set(TalonFXControlMode.PercentOutput, SpeakerSetpoint);
    }
    public void muzzleIntake(){
        ShooterL.set(TalonFXControlMode.PercentOutput, SpeakerSetpoint);
        ShooterR.set(TalonFXControlMode.PercentOutput, -SpeakerSetpoint);
    }
    @Override
    public void periodic(){
        System.out.println("periodic setpoint: " + CurrentSetpoint);
        //DummyMotor.set((ShooterPeriodic.calculate(DummyMotor.getSelectedSensorPosition(), CurrentSetpoint)));
    }
    @Override
    public SubsystemBase asSubsystem() {
        return this;
    }
    
}

