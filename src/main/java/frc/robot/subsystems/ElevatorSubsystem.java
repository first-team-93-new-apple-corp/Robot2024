package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    static DigitalInput topLimit;
    static DigitalInput bottomLimit;
    TalonFX m_motor;
    TalonFXConfiguration m_motorconfig;
    double output;
    PIDController pid = new PIDController(0.0005, 0, 0);
    // AnalogInput m_encoder;
    AnalogEncoder m_encoder;
    double zeroPos;
    double rawoutput;
    public ElevatorSubsystem() {
        m_motor = new TalonFX(Constants.CTRE.RIO.Elevator, "rio");
        m_motorconfig = new TalonFXConfiguration();
        // m_motorconfig.Slot0.kG = 0.36;
        // m_motorconfig.Slot0.kV = 2.38;
        // m_motorconfig.Slot0.kA = 0.05;
        // m_motorconfig.Slot0.kP = 0.05;
        // m_motorconfig.Slot0.kI = 0;
        // m_motorconfig.Slot0.kD = 0;
        m_motor.getConfigurator().apply(m_motorconfig);
        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void initOnce() {
        topLimit = new DigitalInput(0);
        bottomLimit = new DigitalInput(1);
        m_encoder = new AnalogEncoder(0);
        zeroPos = m_encoder.getAbsolutePosition();
    }

    public boolean topLimitTriggered() {
        return topLimit.get();
    }

    public boolean bottomLimitTriggered() {
        return bottomLimit.get();
    }

    //Safety feature. When we are close to the end of the elevator's travel, we want to slow it down so that it doesn't ram into the limit switches.
    public double closeToEndpoint(double speed) {
        /* 
         * Input speed/output value
         * 
         *If speed is negative (less than 0), meaning we are going out, and we are close to the end (motor position is past -65)
         * we want to slow the max speed down to a low value to prevent damage from occuring to the elevator
         * 
         * Changes speed to within limits
         * 
         * Else if speed positive (greater than 0), meaning we are going down, and we are close to the end of our travel (encoder is not past -10), we want to again cap the max speed
         *  to prevent any damage from happening
         * 
         * Changes speed to within limits
         * 
         * Returns speed
         * 
         */
        if (speed < 0 && m_motor.getPosition().getValueAsDouble() < -60) {
            speed = MathUtil.clamp(speed, -0.01, 0.01);
        } else if (speed > 0 && m_motor.getPosition().getValueAsDouble() > -15) {
            speed = MathUtil.clamp(speed, -0.01, 0.01);
        }
        return speed;
    }

    // public double checkEncoder(double speed) {
    //     if (speed < 0 && m_encoder.get() < -65) {
    //         speed = MathUtil.clamp(speed, -0.2, 0.2);
    //     } else if (speed > 0 && m_encoder.get() > -10) {
    //         speed = MathUtil.clamp(speed, -0.15, 0.15);
    //     }
    //     return speed;
    // }

    public void runMotor(double speed) {
        speed = closeToEndpoint(speed);
        speed = capSpeed(speed);
        speed = checkLimits(speed);
        m_motor.set(speed);
    }

    public void toSetpoint(double setpoint) {
        output = pid.calculate(m_motor.getPosition().getValueAsDouble(), setpoint);
        rawoutput = pid.calculate(m_motor.getPosition().getValueAsDouble(), setpoint);
        
        if (setpoint > 1) {
            setpoint = 1;
        } else if (setpoint < -67) {
            setpoint = -67;
        }
        if (-0.03 < output && output < 0.03) {
            output = 0;
        }
        output = closeToEndpoint(output);
        output = capSpeed(output);
        output = checkLimits(output);
        runMotor(output);
    }

    public double checkLimits(double speed) {
        /* Input speed/output
         * 
         * if we are hitting our top limit switch (the uh oh switch) while trying to travel out further (speed negative), 
         * change our speed to zero so that we don't end up running it off the end
         * 
         * Changes speed if it is negative to zero
         * 
         * if we are hitting our bottom limit switch while also trying to go down (speed positive),
         * set our speed to zero again so that we don't break the robot. again.
         * 
         * Changes speed if it is positive to zero
         * 
         * returns speed/output value
         * 
         */
        if (topLimitTriggered()) {
            // if negative (going up) while hitting limit, don't
            if (speed < 0) {
                speed = 0;
            }
            m_motor.set(0);
        }
        if (bottomLimitTriggered()) {
            // if positive (going down) while hitting limt, don't
            if (speed > 0) {
                speed = 0;
            }
            m_motor.set(0);
        }
        return speed;
    }
    public void checkLimits() {
       /*
        * THIS IS DIFFERENT FROM CHECKLIMITS(DOUBLE SPEED)!!!!!
        * if we are hitting a limit switch, kill the motors (likely will not be used too much, but a great safety)
        */
        if (topLimitTriggered()) {
            m_motor.set(0);
        }
        if (bottomLimitTriggered()) {
            m_motor.set(0);
        }
    }

    public void toAmp() {
        toSetpoint(-76);
    }

    public void toSource() {
        toSetpoint(-60);
    }

    public void goDown() {
        toSetpoint(0);
    }

    public void zero() {
        if (!bottomLimitTriggered()) {
            runMotor(0.05);
        } else {
            runMotor(0);
        }
    }

    public double capSpeed(double speed) {
        return MathUtil.clamp(speed, -0.05, 0.05);
    }

    @Override
    public void periodic() {
        // checkLimits();
        SmartDashboard.putNumber("Elevator Pos", m_motor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Top Limit?", topLimitTriggered());
        SmartDashboard.putBoolean("Bottom Limit?", bottomLimitTriggered());
        SmartDashboard.putNumber("Elevator PID Output", output);
        SmartDashboard.putNumber("Raw Out", rawoutput);
        if (!(bottomLimit == null)) {
            if (bottomLimitTriggered()) {
                m_motor.setPosition(0);
                
            }
        }

        if (!(m_encoder == null)) {
            SmartDashboard.putNumber("Abs Enc", m_encoder.get());
        }
    }
}
