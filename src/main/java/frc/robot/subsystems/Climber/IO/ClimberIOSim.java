package frc.robot.subsystems.Climber.IO;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Utilities.MotorSim;


public class ClimberIOSim implements ClimberIO {
    private MotorSim climberLeft;
    private MotorSim climberRight;
    private double maxHeight;

    public ClimberIOSim(ClimberConstants constants) {
        //Revolutions per second
        climberLeft = new MotorSim(100);
        climberRight = new MotorSim(100);
        maxHeight = constants.maxHeight;
        
    }
    @Override
    public void updateValues(ClimberIOInputs inputs){
        climberLeft.periodic();
        climberRight.periodic();
    }

    @Override
    public void leftSpeed(double speed){
        climberLeft.set(speed);
    }

    @Override
    public void rightSpeed(double speed){
        climberRight.set(speed);
    }

    @Override
    public void zeroLeft(){
        climberLeft.setPosition(0);
    }

    @Override
    public void zeroRight(){
        climberRight.setPosition(0);
    }

    @Override
    public double getLeftDraw() {
        return climberLeft.getVelocity() + 300 * -0.5;
    }
    
    @Override
    public double getRightDraw() {
        return climberRight.getVelocity() + 300 * -0.5;
    }

    @Override
    public double leftPosition() {
        return climberLeft.getDistance();
    }

    @Override
    public double rightPosition() {
        return climberRight.getDistance();
    }

    @Override
    public double checkLeftBound(double output) {
        if(output < 0 && climberLeft.getDistance() < -maxHeight) {
            output = 0;
        } else if(output > 0 && climberLeft.getDistance() > -3) {
            output = 0;
        }
        return output;
    }

    @Override
    public double checkRightBound(double output) {
        if(output < 0 && climberRight.getDistance() < -maxHeight) {
            output = 0;
        } else if(output > 0 && climberRight.getDistance() > -3) {
            output = 0;
        }
        return output;
    }

}
