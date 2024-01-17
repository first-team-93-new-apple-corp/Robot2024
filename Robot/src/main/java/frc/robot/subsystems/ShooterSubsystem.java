package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Interfaces.IShooter;

public class ShooterSubsystem extends SubsystemBase implements IShooter{
    //Initations here later
    double CurrentSpeed;
    double ShooterSpeed = 0;
    final double IntakeSetPoint = -1;

    TalonFX ShooterL= new TalonFX(11);
    TalonFX ShooterR= new TalonFX(05);
    CANSparkMax NEOIntake = new CANSparkMax(3, MotorType.kBrushless);
    XboxController xjs = new XboxController(0);
    @Override
    public void intake() {
        CurrentSpeed = IntakeSetPoint;
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
        CurrentSpeed = ShooterSpeed;
        System.out.println("shootAmp at lower speed");
        ShooterL.set(TalonFXControlMode.PercentOutput, -ShooterSpeed);
        ShooterR.set(TalonFXControlMode.PercentOutput, ShooterSpeed);
    }
    @Override
    public void shootSpeaker() {
        CurrentSpeed = ShooterSpeed;
        System.out.println("shootSpeaker at high speed");
        ShooterL.set(TalonFXControlMode.PercentOutput, -ShooterSpeed);
        ShooterR.set(TalonFXControlMode.PercentOutput, ShooterSpeed);
    }
    public void muzzleIntake(){
        ShooterL.set(TalonFXControlMode.PercentOutput, ShooterSpeed);
        ShooterR.set(TalonFXControlMode.PercentOutput, -ShooterSpeed);
    }
    public void shootSpeedPlus(){
        ShooterSpeed += 0.05;
        CurrentSpeed = ShooterSpeed;
        SmartDashboard.putNumber("CurrentShooterSpeed", ShooterSpeed);
    }
    public void shootSpeedMinus(){
        ShooterSpeed -= 0.05;
        CurrentSpeed = ShooterSpeed;
        SmartDashboard.putNumber("CurrentShooterSpeed", ShooterSpeed);
    }
    @Override
    public void periodic(){
        System.out.println("periodic setpoint: " + CurrentSpeed);
        //DummyMotor.set((ShooterPeriodic.calculate(DummyMotor.getSelectedSensorPosition(), CurrentSetpoint)));
    }
    @Override
    public SubsystemBase asSubsystem() {
        return this;
    }
    
}

