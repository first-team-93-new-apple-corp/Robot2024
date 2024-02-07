package frc.robot.subsystems;

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

    @Override
    public void periodic() {

    }
}
