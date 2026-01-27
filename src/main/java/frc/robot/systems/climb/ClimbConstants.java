package frc.robot.systems.climb;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.RotationLimitMotorHardware;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;

public class ClimbConstants {
    public static final RotationLimitMotorHardware kClimbMotorConstants = new RotationLimitMotorHardware(
        42, // Motor ID // TODO: TUNE ME!
        Constants.kSubsystemsCANBus, 
        1, // Rotor to Mechanism Ratio // TODO: TUNE ME!
        InvertedValue.CounterClockwise_Positive,
        NeutralModeValue.Coast,
        new CurrentLimits(30, 40),
        new RotationSoftLimits(
            Rotation2d.fromRotations(0), 
            Rotation2d.fromRotations(0)
        )
    );
}
