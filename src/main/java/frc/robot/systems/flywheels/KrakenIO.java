package frc.robot.systems.flywheels;

import static frc.robot.systems.drive.DriveConstants.kCANBus;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.systems.drive.DriveConstants;
import frc.robot.systems.drive.modules.ModuleIO.ModuleInputs;
import frc.robot.systems.flywheels.FlywheelConstants.FlywheelHardwareConfiguration;

public class KrakenIO implements FlywheelIO{
    private final TalonFX mFlywheelTopMotor;
    private StatusSignal<Voltage> mFlywheelTopMotorVoltage;
    public StatusSignal<AngularVelocity> mFlywheelTopMotorVelocity;


    private final TalonFX mFlywheelBottomMotor;
    private StatusSignal<Voltage> mFlywheelBottomMotorVoltage;
    public StatusSignal<AngularVelocity> mFlywheelBottomMotorVelocity;


    public KrakenIO(FlywheelHardwareConfiguration pFlywheelHardwareConfiguration){

        /*TODO: check if I need to add anything more for the configs! Also should I init the other logging inputs? What are the main things I need?*/
        
        /*TOP MOTOR CONFIGS */
        mFlywheelTopMotor = new TalonFX(pFlywheelHardwareConfiguration.kMotorID(), pFlywheelHardwareConfiguration.kCanBus());
        TalonFXConfiguration mTopMotorConfig = new TalonFXConfiguration();
        mTopMotorConfig.CurrentLimits.StatorCurrentLimit = FlywheelConstants.kSmartCurrentLimit;
        mTopMotorConfig.Voltage.PeakForwardVoltage = FlywheelConstants.kPeakVoltage;
        mTopMotorConfig.Voltage.PeakReverseVoltage = -FlywheelConstants.kPeakVoltage;
        mTopMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mTopMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        mFlywheelTopMotor.getConfigurator().apply(mTopMotorConfig);

        mFlywheelTopMotorVoltage = mFlywheelTopMotor.getMotorVoltage();
        mFlywheelTopMotorVelocity = mFlywheelTopMotor.getVelocity();


        /*BOTTOM MOTOR CONFIGS */
        mFlywheelBottomMotor = new TalonFX(pFlywheelHardwareConfiguration.kMotorID(), pFlywheelHardwareConfiguration.kCanBus());
        TalonFXConfiguration mBottomMotorConfig = new TalonFXConfiguration();
        mBottomMotorConfig.CurrentLimits.StatorCurrentLimit = FlywheelConstants.kSmartCurrentLimit;
        mBottomMotorConfig.Voltage.PeakForwardVoltage = FlywheelConstants.kPeakVoltage;
        mBottomMotorConfig.Voltage.PeakReverseVoltage = -FlywheelConstants.kPeakVoltage;
        mBottomMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mBottomMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        mFlywheelBottomMotor.getConfigurator().apply(mBottomMotorConfig);

        mFlywheelBottomMotorVoltage = mFlywheelBottomMotor.getMotorVoltage();
        mFlywheelBottomMotorVelocity = mFlywheelBottomMotor.getVelocity();

    }

    @Override
    public void updateInputs(FlywheelInputs inputs) {
        /*TODO: init more values to log */
        inputs.iFlywheelTopMotorVolts = mFlywheelTopMotorVoltage.getValueAsDouble();
        inputs.iFlywheelTopVelocityMPS = mFlywheelTopMotorVelocity.getValueAsDouble();

        inputs.iFlywheelBottomMotorVolts = mFlywheelBottomMotorVoltage.getValueAsDouble();
        inputs.iFlywheelBottomVelocityMPS = mFlywheelBottomMotorVelocity.getValueAsDouble();

    }

 
    @Override
    public void setTopFlywheeVolts(double volts) {
        mFlywheelTopMotor.setVoltage(volts);
    }

    //TODO: do flywheel PID's later
    @Override
    public void setTopFlywheePID(double kP, double kI, double kD) {}
    
    @Override
    public void setBottomFlywheeVolts(double volts) {
        mFlywheelBottomMotor.setVoltage(volts);
    }

    //TODO: do flywheel PID's later
    @Override
    public void setBottomFlywheePID(double kP, double kI, double kD) {}


    


}
