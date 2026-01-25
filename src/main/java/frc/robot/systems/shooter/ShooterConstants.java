package frc.robot.systems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.lib.hardware.Records.FollowerMotorHardware;
import frc.lib.hardware.Records.InternalMotorHardware;

public class ShooterConstants {
    public static class Flywheels {
        final InternalMotorHardware kFlywheelLeaderConfig = new InternalMotorHardware(
            54,
            Constants.kSubsystemsCANBus,
            1,
            InvertedValue.CounterClockwise_Positive,
            NeutralModeValue.Coast,
            30,
            40
        );

        final FollowerMotorHardware kFlywheelFollowerConfig = new FollowerMotorHardware(
            55,
            kFlywheelLeaderConfig,
            MotorAlignmentValue.Opposed
        );
    }
}
