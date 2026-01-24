package frc.robot.systems.flywheels;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.flywheels.FlywheelConstants.FlywheelSetpoint;

public class Flywheel extends SubsystemBase{

    /* LEFT MOTOR LOGGING */
        /*ProfiledPIDController logging */
        public static final LoggedTunableNumber leftFlywheelMotorP = new LoggedTunableNumber("Flywheel/Left/kP", FlywheelConstants.LeftControlConfig.motorController().getP());
        public static final LoggedTunableNumber leftFlywheelMotorI = new LoggedTunableNumber("Flywheel/Left/kP", FlywheelConstants.LeftControlConfig.motorController().getI());
        public static final LoggedTunableNumber leftFlywheelMotorD = new LoggedTunableNumber("Flywheel/Left/kD", FlywheelConstants.LeftControlConfig.motorController().getD());
        public static final LoggedTunableNumber leftFlywheelMotorVMax = new LoggedTunableNumber("Flywheel/Left/kVMax", FlywheelConstants.LeftControlConfig.motorController().getConstraints().maxVelocity);
        public static final LoggedTunableNumber leftFlywheelMotorAMax = new LoggedTunableNumber("Flywheel/Left/kAMax", FlywheelConstants.LeftControlConfig.motorController().getConstraints().maxAcceleration);
        /*FF Logging */
        public static final LoggedTunableNumber leftFlywheelMotorS = new LoggedTunableNumber("Flywheel/Left/kS", FlywheelConstants.LeftControlConfig.motorFF().getKs());
        public static final LoggedTunableNumber leftFlywheelMotorV = new LoggedTunableNumber("Flywheel/Left/kV", FlywheelConstants.LeftControlConfig.motorFF().getKv());
        public static final LoggedTunableNumber leftFlywheelMotorA = new LoggedTunableNumber("Flywheel/Left/kA", FlywheelConstants.LeftControlConfig.motorFF().getKa());

    /* RIGHT MOTORLOGGING */
        /*ProfiledPIDController logging */
        public static final LoggedTunableNumber rightFlywheelMotorP = new LoggedTunableNumber("Flywheel/Right/kP", FlywheelConstants.RightControlConfig.motorController().getP());
        public static final LoggedTunableNumber rightFlywheelMotorI = new LoggedTunableNumber("Flywheel/Right/kP", FlywheelConstants.RightControlConfig.motorController().getI());
        public static final LoggedTunableNumber rightFlywheelMotorD = new LoggedTunableNumber("Flywheel/Right/kD", FlywheelConstants.RightControlConfig.motorController().getD());
        public static final LoggedTunableNumber rightFlywheelMotorVMax = new LoggedTunableNumber("Flywheel/Right/kVMax", FlywheelConstants.RightControlConfig.motorController().getConstraints().maxVelocity);
        public static final LoggedTunableNumber rightFlywheelMotorAMax = new LoggedTunableNumber("Flywheel/Right/kAMax", FlywheelConstants.RightControlConfig.motorController().getConstraints().maxAcceleration);
        /*FF Logging */
        public static final LoggedTunableNumber rightFlywheelMotorS = new LoggedTunableNumber("Flywheel/Right/kS", FlywheelConstants.RightControlConfig.motorFF().getKs());
        public static final LoggedTunableNumber rightFlywheelMotorV = new LoggedTunableNumber("Flywheel/Right/kV", FlywheelConstants.RightControlConfig.motorFF().getKv());
        public static final LoggedTunableNumber rightFlywheelMotorA = new LoggedTunableNumber("Flywheel/Right/kA", FlywheelConstants.RightControlConfig.motorFF().getKa());


    private FlywheelIO io;
    private FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();
    private SimpleMotorFeedforward mLeftFF = FlywheelConstants.LeftControlConfig.motorFF();
    private SimpleMotorFeedforward mRightFF = FlywheelConstants.RightControlConfig.motorFF();
    
    public Flywheel(FlywheelIO pIo) {
        this.io = pIo;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

         LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    io.setLeftFlywheelPID(leftFlywheelMotorP.get(), 0.0, leftFlywheelMotorD.get(), leftFlywheelMotorVMax.get(), leftFlywheelMotorAMax.get());
                },
                leftFlywheelMotorP,
                leftFlywheelMotorD,
                leftFlywheelMotorVMax,
                leftFlywheelMotorAMax
            );

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                mLeftFF = new SimpleMotorFeedforward(leftFlywheelMotorS.get(), leftFlywheelMotorV.get(), leftFlywheelMotorA.get());
            },
            leftFlywheelMotorS,
            leftFlywheelMotorV,
            leftFlywheelMotorA
        );

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                io.setRightFlywheelPID(rightFlywheelMotorP.get(), 0.0, rightFlywheelMotorD.get(), rightFlywheelMotorVMax.get(), rightFlywheelMotorAMax.get());
            },
            rightFlywheelMotorP,
            rightFlywheelMotorD,
            rightFlywheelMotorVMax,
            rightFlywheelMotorAMax
        );

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                mRightFF = new SimpleMotorFeedforward(rightFlywheelMotorS.get(), rightFlywheelMotorV.get(), rightFlywheelMotorA.get());
            },
            rightFlywheelMotorS,
            rightFlywheelMotorV,
            rightFlywheelMotorA
        );
    }

    public void setLeftFlyWheelVolts(double pVolts){
        io.setLeftFlywheelVolts(pVolts);
    }

    public void setLeftPID(double kP, double kI, double kD, double kV, double kA){
        io.setLeftFlywheelPID(kP, kI, kD, kV, kA);
    }

    public void setLeftVelocity(AngularVelocity setpointRPS, double ff){
        io.setLeftFlywheelVelocity(setpointRPS, ff);
    }
    
    public void setRightFlyWheelVolts(double pVolts){
        io.setRightFlywheelVolts(pVolts);
    }

    public void setRightPID(double kP, double kI, double kD, double kV, double kA){
        io.setRightFlywheelPID(kP, kI, kD, kV, kA);
    }

    public void setRightVelocity(AngularVelocity setpointRPS, double ff){
        io.setRightFlywheelVelocity(setpointRPS, ff);
    }

    public Command shootCmd() {
        return new FunctionalCommand(
            () -> {
                setLeftPID(
                    leftFlywheelMotorP.get(),
                    leftFlywheelMotorI.get(),
                    leftFlywheelMotorD.get(),
                    leftFlywheelMotorVMax.get(),
                    leftFlywheelMotorAMax.get());

                setLeftVelocity(
                    FlywheelConstants.FlywheelSetpoint.Outtake.getRPS(),
                    mLeftFF.calculate(FlywheelConstants.LeftControlConfig.motorController().getSetpoint().velocity));
   

                setRightPID(
                    rightFlywheelMotorP.get(), 
                    rightFlywheelMotorI.get(), 
                    rightFlywheelMotorD.get(), 
                    rightFlywheelMotorVMax.get(), 
                    rightFlywheelMotorAMax.get());

                setRightVelocity(
                    FlywheelConstants.FlywheelSetpoint.Outtake.getRPS(), 
                    mRightFF.calculate(FlywheelConstants.RightControlConfig.motorController().getSetpoint().velocity));
            },

            () -> {},

            (interrupted) -> {
                setLeftFlyWheelVolts(0);
                setRightFlyWheelVolts(0);
            },

            () -> false,
            
            this);
    }
}
