package frc.robot.subsystems;

interface ArmInterface{
    public void toSetpoint(double setpoint);
    public void directMotorCommand(double speed);
    public void stopMotors();
}