package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ClimberSubsystem extends SubsystemBase {
    public TalonFX climberLeft = new TalonFX(26, "drivetrain");
    public TalonFX climberRight = new TalonFX(25, "drivetrain");
    public XboxController opController;
    public JoystickButton opButtonBack;
    public JoystickButton opButtonStart;
    private double lLow = 0;
    private double lHigh = 2048 * 15;
    private double rLow = 0;
    private double rHigh = 2048 * 15;
    private boolean goingDownL;
    private boolean goingDownR;


    private PositionVoltage positionL = new PositionVoltage(0);
    private PositionVoltage positionR = new PositionVoltage(0);

    ClimberSubsystem(
            TalonFX climberRight,
            TalonFX climberLeft,
            JoystickButton opButtonBack,
            JoystickButton opButtonStart,
            XboxController opController) {
        this.climberLeft = climberLeft;
        this.climberRight = climberRight;
        this.opButtonBack = opButtonBack;
        this.opController = opController;
        this.opButtonStart = opButtonStart;

        boolean run = true;
        int stop = 15;
        for (int times = 0; times < 100; times ++) {
            if(climberLeft.getPosition().getValueAsDouble() < 500 && goingDownL == false ) {
                climberLeft.setControl(positionL.withPosition(lHigh));
            } else {
                climberLeft.setControl(positionL.withPosition(lLow));
                goingDownL = true;
            }
            if(climberRight.getPosition().getValueAsDouble() < 500 && goingDownR == false ) {
                climberRight.setControl(positionR.withPosition(rHigh));
            } else {
                climberRight.setControl(positionR.withPosition(rLow));
                goingDownR = true;
            }
        }
        if (opButtonStart.getAsBoolean()) {
            while (run == true) {
                climberLeft.set(.15);
                climberRight.set(.15);
                for (int i = 0, j = -14; i < stop + 1; i++, j++) {
                    if (i == stop && !(j == stop)) {
                        climberLeft.setInverted(true);
                        climberRight.setInverted(true);
                        i = 0;
                    } else if (opButtonBack.getAsBoolean()) {
                        climberLeft.stopMotor();
                        climberRight.stopMotor();
                        run = false;
                    } else if (j == stop) {
                        climberLeft.setInverted(false);
                        climberRight.setInverted(false);
                        j = -14;
                    } else {
                    }
                }
            }
        }
    }

    @Override
    public void periodic() {

    }

}
