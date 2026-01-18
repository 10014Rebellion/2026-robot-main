package frc.robot.systems.flywheels;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

public class FlywheelConstants {

    public static int kTopFlywheelID = 21;
    public static int kBottomFlywheelID = 53;
    public static boolean kInverted = false;

    //TODO: check if I need to add more constants
    /*not sure if these are correct; check later! */
    public static double kSmartCurrentLimit = 60;
    public static String kCanBus = "drive";
    public static double kSecondaryCurrentLimit = 75;
    public static double kPeakVoltage = 12;

    public static FlywheelHardwareConfiguration topMotorConfiguration = new FlywheelHardwareConfiguration(
        kTopFlywheelID,
        kCanBus,
        kSmartCurrentLimit,
        kSecondaryCurrentLimit,  
        kInverted
    );

    public static FlywheelHardwareConfiguration bottomMotorConfiguration = new FlywheelHardwareConfiguration(
        kBottomFlywheelID,
        kCanBus,
        kSmartCurrentLimit,
        kSecondaryCurrentLimit,  
        kInverted
    );
    

    //TODO: should I add more parameters? 
    public record FlywheelHardwareConfiguration(
        int kMotorID,
        String kCanBus,
        double kSmartCurrentLimit, 
        double kSecondaryLimit,
        boolean kInverted
    ){}
    

}
