package frc.robot.subsystems.Shooter.IO;

public interface ShooterIO {

  public static class ShooterIOInputs {
      public double ShooterSpeed;
      public double ShooterLVel;
      public double ShooterRVel;
    }

    public void updateValues(ShooterIOInputs inputs);

    public void shoot(double speed);

    public void prime();

    public void AutonPrime();

    public void shootAmp();

    public void kicker(double KickerSpeed);

    public void intakeFront();

    public void shooterStop();

    public void kickerStop();

    public void ampKicker();

    public void AmpForAuton();

    public void ShootingforAuton();

    public void DribbleOutNote();

    public void increaseSpeed();

    public void decreaseSpeed();

}
