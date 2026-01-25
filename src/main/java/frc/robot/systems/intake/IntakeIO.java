// REBELLION 10014

package frc.robot.systems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeInputs {
      public boolean iIsIntakeConnected = false;
      public double iIntakeVelocityMPS = 0.0;
      public double iIntakeAccelerationMPSS = 0.0;
      public double iIntakeMotorVolts = 0.0;
      public double iIntakeSupplyCurrentAmps = 0.0;
      public double iIntakeStatorCurrentAmps = 0.0;
      public double iIntakeTempCelsius = 0.0;
    }

    public default void updateInputs(IntakeInputs pInputs) {}

    public default void setMotorVolts(double pVolts) {}

    public default void stopMotor() {}

}
