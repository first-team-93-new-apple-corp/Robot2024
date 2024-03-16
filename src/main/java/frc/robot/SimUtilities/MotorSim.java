package frc.robot.SimUtilities;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class MotorSim implements MotorController{
    
    final static double maxVoltage = 12.5;
    final static double period = 0.020; // 20ms
    final double maxVelocity;
    double direction = 1;
    double currentVelocity = 0;
    double AcumulatedDistance = 0;

    public MotorSim() {
        this.maxVelocity = 1;
    }

    public MotorSim(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    public MotorSim(double maxVelocity, double startingDistance) {
        this.maxVelocity = maxVelocity;
        this.AcumulatedDistance = startingDistance;
    }

    @Override
    public void set(double speed){
        currentVelocity = speed * maxVelocity * direction;

    }

    @Override
    public double get() {
        return currentVelocity/maxVelocity;
    }

    @Override
    public void setInverted(boolean isInverted)  {
        direction = isInverted ? -1 : 1;
    }

    @Override
    public boolean getInverted(){
        return direction < 0;
    }

    @Override
    public void disable() {
        stopMotor();
    }

    @Override
    public void stopMotor() {
        currentVelocity = 0;
    }

    public void setPosition(double distance) {
        AcumulatedDistance = distance;
    }

    public double getDistance() {
        return AcumulatedDistance;
    }

    public double getVelocity() {
        return currentVelocity;
    }

    @Override
    public void setVoltage(double outputVolts) {
        set(outputVolts / maxVoltage);
    }

    public double getVoltage(){
        return get()*maxVoltage;
    }

    public void periodic() {
        AcumulatedDistance += currentVelocity * period;
    }

}