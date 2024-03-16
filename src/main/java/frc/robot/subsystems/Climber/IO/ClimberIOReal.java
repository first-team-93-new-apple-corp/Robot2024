package frc.robot.subsystems.Climber.IO;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ClimberConstants;


public class ClimberIOReal implements ClimberIO {
    public TalonFX climberLeft;
    public TalonFX climberRight;
    public XboxController op;
    public double maxHeight;

    public ClimberIOReal(ClimberConstants constants, XboxController op) {
        climberLeft = new TalonFX(constants.leftClimber);
        climberRight = new TalonFX(constants.rightClimber);
        this.op = op;
        maxHeight = constants.maxHeight;
    }
    @Override
    public void updateValues(ClimberIOInputs inputs){

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
        return climberLeft.getSupplyCurrent().getValueAsDouble();
    }
    
    @Override
    public double getRightDraw() {
        return climberRight.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double leftPosition() {
        return climberLeft.getPosition().getValueAsDouble();
    }

    @Override
    public double rightPosition() {
        return climberRight.getPosition().getValueAsDouble();
    }

    @Override
    public double checkLeftBound(double output) {
        if(output < 0 && climberLeft.getPosition().getValueAsDouble() < -maxHeight) {
            output = 0;
        } else if(output > 0 && climberLeft.getPosition().getValueAsDouble() > -3) {
            output = 0;
        }
        return output;
    }

    @Override
    public double checkRightBound(double output) {
        if(output < 0 && climberRight.getPosition().getValueAsDouble() < -maxHeight) {
            output = 0;
        } else if(output > 0 && climberRight.getPosition().getValueAsDouble() > -3) {
            output = 0;
        }
        return output;
    }

}
