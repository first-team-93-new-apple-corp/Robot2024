package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase implements IShooter{
    //Initations here later
    double CurrentSetpoint;
    final double DummyAmpSetPoint = 65;
    final double DummyIntakeSetPoint = 7;
    final double DummySpeakerSetpoint = 0;
    //PIDController ShooterPeriodic = new PIDController(0, 0, 0);
    //WPI_TalonFX DummyMotor = new WPI_TalonFX(00);
    
    @Override
    public void intake() {
        CurrentSetpoint = DummyIntakeSetPoint;
        System.out.println("Muzzle loading");
    }
    @Override
    public void stop() {
        System.out.println("Stop shooting motors");
    
    }
    @Override
    public void shootAmp() {
        CurrentSetpoint = DummyAmpSetPoint;
        System.out.println("shootAmp at lower speed");
    }
    @Override
    public void shootSpeaker() {
        CurrentSetpoint = DummySpeakerSetpoint;
        System.out.println("shootSpeaker at high speed");
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

