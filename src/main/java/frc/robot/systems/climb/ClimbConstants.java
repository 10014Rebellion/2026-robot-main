package frc.robot.systems.climb;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.Constants;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.ElevatorController;
import frc.lib.hardware.HardwareRecords.MotionConstraints;
import frc.lib.hardware.HardwareRecords.PDConstants;


public class ClimbConstants {
    public static final BasicMotorHardware kClimbMotorConstants = new BasicMotorHardware(
        42, // Motor ID // TODO: TUNE ME!
        Constants.kSubsystemsCANBus, 
        1, // Rotor to Mechanism Ratio // TODO: TUNE ME!
        InvertedValue.CounterClockwise_Positive,
        NeutralModeValue.Coast,
        new CurrentLimits(30, 40)
    );

    public static final ElevatorController kController0 = new ElevatorController(
        0, 
        new PDConstants(0, 0), 
        new ElevatorFeedforward(0, 0, 0),
        new MotionConstraints(0, 0, 0));

    public static final ElevatorController kController1 = new ElevatorController(
        1, 
        new PDConstants(0, 0), 
        new ElevatorFeedforward(0, 0, 0),
        new MotionConstraints(0, 0, 0));

    public static final ElevatorController kController2 = new ElevatorController(
        2, 
        new PDConstants(0, 0), 
        new ElevatorFeedforward(0, 0, 0),
        new MotionConstraints(0, 0, 0));

    public static final ElevatorController kController3 = new ElevatorController(
        3, 
        new PDConstants(0, 0), 
        new ElevatorFeedforward(0, 0, 0),
        new MotionConstraints(0, 0, 0));
}
