package frc.robot.systems.hood;

import com.ctre.phoenix6.signals.NeutralModeValue;



public class HoodConstants {

    // flywheel motor CAN ID
    public static final int kHoodMotorID = 15; // TODO: TUNE ME

    // flywheels should coast when stopped
    //public static final NeutralMode kNeutralMode = NeutralMode.Coast;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;

    // controls how fast and accurate the flywheel reaches speed
    public static final double kP = 0.001; // TODO: TUNE ME
    public static final double kI = 0.0;
    public static final double kD = 0.0;


    // help the flywheel hold speed
    public static final double kS = 0.0; // TODO: TUNE ME
    public static final double kV = 0.0; // TODO: TUNE ME
    public static final double kA = 0.0; // usually unused
    public static final double kG = 0.0;

    // max RPM of the flywheel
    public static final double kMaxRPM = 5000; // TODO: TUNE ME

    // current limit to protect motor
    public static final int kCurrentLimit = 60;
}