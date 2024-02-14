package frc.robot.subsystems.Code24;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    public TalonFX climberLeft;
    public TalonFX climberRight;
    public XboxController op;

    public ClimberSubsystem(XboxController op) {
        this.op = op;
        climberLeft = new TalonFX(Constants.CTRE.RIO.L_Climber, "rio");
        climberRight = new TalonFX(Constants.CTRE.RIO.R_Climber, "rio");
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

    @Override
    public void periodic() {

    }
}
