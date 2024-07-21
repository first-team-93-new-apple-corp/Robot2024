package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class IntakeSubsystem {
    public TalonFX intakeMotor;

    private double intakeSpeed = 0.75;
    private double shootSpeed = .80;
    private double ampSpeed = 0.4;
    private TimeOfFlight TOF;
    public IntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.CTRE.Intake);
        intakeMotor.setInverted(true);
        TOF = new TimeOfFlight(Constants.Sensors.CAN.TOF);
        TOF.setRangingMode(RangingMode.Short, 24);
    }
    public void set(double in) {
        intakeMotor.set(in);
    }
    public void stop() {
        intakeMotor.set(0);
    }
    public void intake() {
        intakeMotor.set(intakeSpeed);
    }
    public void shoot() {
        intakeMotor.set(shootSpeed);
    }
    public void revShoot(){
        intakeMotor.set(-0.1);
    }
    public void amp() {
        intakeMotor.set(ampSpeed);
    }
    public boolean hasNote() {
        SmartDashboard.putNumber("TOF DS", TOF.getRange());
        return TOF.getRange() < 100;
    }
}
