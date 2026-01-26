package frc.robot.systems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.lib.hardware.HardwareRecords.InternalMotorHardware;

public class IntakeConstants {
    public static final InternalMotorHardware kIntakeMotorConstants = new InternalMotorHardware(
        41,
        Constants.kSubsystemsCANBus,
        1,
        InvertedValue.CounterClockwise_Positive,
        NeutralModeValue.Coast,
        30,
        40
    );
    
}
