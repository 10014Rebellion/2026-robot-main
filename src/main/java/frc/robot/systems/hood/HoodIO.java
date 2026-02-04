package frc.robot.systems.hood;


/** The Hood subsystem's hardware interface. */
public interface HoodIO {

  public static class HoodIOInputs {

    // Whether the motor is connected and responding
    public boolean iIsHoodConnected = false;
    public double iHoodVelocityMPS = 0.0;
    public double iHoodAccelerationMPSS = 0.0;
    public double iHoodMotorVolts = 0.0;
    public double iHoodSupplyCurrentAmps = 0.0;
    public double iHoodStatorCurrentAmps = 0.0;
    public double iHoodTempCelsius = 0.0;
  }

  
  public default void updateInputs(HoodIOInputs inputs) {}

  /**
   * Applies a voltage to the Hood motor.
   *
   * @param volts Voltage from -12 to +12
   */
  public default void setVoltage(double volts) {}

  /**
   * Stops the Hood motor.
   * For TalonFX, this usually sets the motor to neutral.
   */
  public default void stop() {}

  public default void setDrivePID(int pSlot, double pKP, double pKI, double pKD) {}

  public default void setFeedforward(double kS, double kV, double kA, double kG) {}

  public default void setMotorPosition(double pPositionM, double pFeedforward){}
}

