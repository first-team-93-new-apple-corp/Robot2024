package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    public TalonFX climberLeft;
    public TalonFX climberRight;
    public XboxController op;
    public double maxHeight = 160;
    public ClimberSubsystem(XboxController op) {
        this.op = op;
        climberLeft = new TalonFX(Constants.CTRE.RIO.L_Climber, "drivetrain");
        climberRight = new TalonFX(Constants.CTRE.RIO.R_Climber, "drivetrain");
        climberLeft.setNeutralMode(NeutralModeValue.Brake);
        climberRight.setNeutralMode(NeutralModeValue.Brake);
    }
    
    public void leftSpeed(double speed) {
        climberLeft.set(speed);
    }
    public void rightSpeed(double speed) {
        climberRight.set(speed);
    }
    public void zeroLeft() {
        climberLeft.setPosition(0);
    }
    public void zeroRight() {
        climberRight.setPosition(0);
    }
    public double getLeftDraw() {
        return climberLeft.getSupplyCurrent().getValueAsDouble();
    }
    public double getRightDraw() {
        return climberRight.getSupplyCurrent().getValueAsDouble();
    }
    public double leftPosition() {
        return climberLeft.getPosition().getValueAsDouble();
    }
    public double rightPosition() {
        return climberRight.getPosition().getValueAsDouble();
    }
    public TalonFX getLeft() {
        return climberLeft;
    }
    public TalonFX getRight() {
        return climberRight;
    }
    public double checkLeftBound(double output) {
        if(output < 0 && climberLeft.getPosition().getValueAsDouble() < -maxHeight) {
            output = 0;
        } else if(output > 0 && climberLeft.getPosition().getValueAsDouble() > -3) {
            output = 0;
        }
        return output;
    }
    public double checkRightBound(double output) {
        if(output < 0 && climberRight.getPosition().getValueAsDouble() < -maxHeight) {
            output = 0;
        } else if(output > 0 && climberRight.getPosition().getValueAsDouble() > -3) {
            output = 0;
        }
        return output;
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("L_POS", climberLeft.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("R_POS", climberRight.getPosition().getValueAsDouble());
    }
}
