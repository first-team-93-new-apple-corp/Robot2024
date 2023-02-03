package frc.robot.subsystems;


interface GenericMotorSubsystem {
    public void directMotorCommand(double speed);
    public void toSetpoint(double setpoint);
    public boolean atSetpoint();
    public void stopMotors();
}