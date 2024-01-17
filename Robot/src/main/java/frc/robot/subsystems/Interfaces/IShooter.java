package frc.robot.subsystems.Interfaces;

public interface IShooter extends ISubsystem{
    public void intake();
    public void muzzleIntake();
    public void stop();
    public void shootAmp();
    public void shootSpeaker();
    public void shootSpeedMinus();
    public void shootSpeedPlus();
}
