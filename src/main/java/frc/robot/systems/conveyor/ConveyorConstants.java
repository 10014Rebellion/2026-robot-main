package frc.robot.systems.conveyor;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.lib.hardware.HardwareRecords.InternalMotorHardware;

public class ConveyorConstants {
    public static final InternalMotorHardware kConveyorMotorConstants = new InternalMotorHardware(
        42, // Motor ID // TODO: TUNE ME!
        Constants.kSubsystemsCANBus, 
        1, // Rotor to Mechanism Ratio // TODO: TUNE ME!
        InvertedValue.CounterClockwise_Positive,
        NeutralModeValue.Coast,
        30, // Supply Current Limit // TODO: TUNE ME!
        40 // Stator Current Limit // TODO: TUNE ME!
    );
}
