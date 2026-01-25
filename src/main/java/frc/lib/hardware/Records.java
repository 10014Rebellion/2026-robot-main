package frc.lib.hardware;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Records {
    public static record InternalMotorHardware(
        int motorID, 
        CANBus canBus, 
        double rotorToMechanismRatio, 
        InvertedValue direction,
        NeutralModeValue neutralMode,
        int supplyCurrentLimit,
        int statorCurrentLimit
    ) {}

    public static record FollowerMotorHardware(
        int motorID,
        InternalMotorHardware leaderConfig,
        MotorAlignmentValue alignmentValue
    ) {}

    public static record CANCoderHardware(
        int cancoderID,
        double cancoderToMechanismRatio
    ) {}

    public static record ClosedSimpleMotorHardware(
        InternalMotorHardware internalMotor,
        CANCoderHardware feedbackSensor,
        SimpleController controller
    ) {}

    public static record ClosedElevatorMotorHardware(
        InternalMotorHardware internalMotor,
        CANCoderHardware feedbackSensor,
        SimpleController controller,
        double forwardLimitM,
        double backwardLimitM
    ) {}

    public static record ClosedArmMotorHardware(
        InternalMotorHardware internalMotor,
        CANCoderHardware feedbackSensor,
        SimpleController controller,
        double forwardLimitDeg,
        double backwardLimitDeg
    ) {}

    public static record ElevatorController(
        double slot,
        double kP,
        double kD,
        ElevatorFeedforward kFeedforward
    ) {}

    public static record ArmController(
        double slot,
        double kP,
        double kD,
        ArmFeedforward kFeedforward
    ) {}

    public static record SimpleController(
        double slot,
        double kP,
        double kD,
        SimpleMotorFeedforward kFeedforward
    ) {}
}
