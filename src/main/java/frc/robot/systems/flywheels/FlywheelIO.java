package frc.robot.systems.flywheels;

import org.littletonrobotics.junction.AutoLog;


public interface FlywheelIO {

    @AutoLog
    public static class FlywheelInputs {

    /*TODO: update the values I don't initilize later! */
    /*Let me know if I should add more for logging. Just base values for now */
    public double iFlywheelTopVelocityMPS = 0.0;
    public double iFlywheelTopStatorCurrentAmps = 0.0;
    public double iFlywheelTopSupplyCurrentAmps = 0.0;
    public double iFlywheelTopTorqueCurrentAmps = 0.0;
    public double iFlywheelTopTemperatureCelsius = 0.0;
    public double iFlywheelTopAppliedVolts = 0.0;
    public double iFlywheelTopMotorVolts = 0.0;
    public double iFlywheelTopAccelerationMPSS = 0.0;

    public double iFlywheelBottomVelocityMPS = 0.0;
    public double iFlywheelBottomStatorCurrentAmps = 0.0;
    public double iFlywheelBottomSupplyCurrentAmps = 0.0;
    public double iFlywheelBottomTorqueCurrentAmps = 0.0;
    public double iFlywheelBottomTemperatureCelsius = 0.0;
    public double iFlywheelBottomAppliedVolts = 0.0;
    public double iFlywheelBottomMotorVolts = 0.0;
    public double iFlywheelBottomAccelerationMPSS = 0.0;

    }

    /*TODO: DO I need more methods? */
    public default void updateInputs(FlywheelInputs inputs) {}

    public default void setTopFlywheeVolts(double volts) {}

    public default void setTopFlywheePID(double kP, double kI, double kD) {}

    public default void setBottomFlywheeVolts(double volts) {}

    public default void setBottomFlywheePID(double kP, double kI, double kD) {}

}
