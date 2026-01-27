package frc.robot.systems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface HoodIO {
  @AutoLog
  public static class HoodInputs {
    public boolean iIsHoodConnected = false;
    public String iHoodControlMode = "";
    public Rotation2d iHoodAngle = Rotation2d.kZero;
    public double iHoodVelocityRPS = 0.0;
    public double iHoodAccelerationRPSS = 0.0;
    public double iHoodMotorVolts = 0.0;
    public double iHoodSupplyCurrentAmps = 0.0;
    public double iHoodStatorCurrentAmps = 0.0;
    public double iHoodTempCelsius = 0.0;
  }

  public default void updateInputs(HoodInputs pInputs) {}

  public default void setMotorPosition(double pVelocityRPS, double pFeedforward) {}

  public default void setMotorVolts(double pVolts) {}

  public default void stopMotor() {}
}
