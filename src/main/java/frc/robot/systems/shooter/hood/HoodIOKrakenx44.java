package frc.robot.systems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.Records.InternalMotorHardware;

public class HoodIOKrakenx44 implements HoodIO{
    private final TalonFX mHoodMotor;
    private final VoltageOut mHoodVoltageControl = new VoltageOut(0.0);
    private final PositionDutyCycle mHoodPositionControl = new PositionDutyCycle(0.0);
    private final StatusSignal<ControlModeValue> mHoodControlMode;
    private final StatusSignal<AngularVelocity> mHoodVelocityRPS;
    private final StatusSignal<Voltage> mHoodVoltage;
    private final StatusSignal<Current> mHoodSupplyCurrent;
    private final StatusSignal<Current> mHoodStatorCurrent;
    private final StatusSignal<Temperature> mHoodTempCelsius;
    private final StatusSignal<AngularAcceleration> mHoodAccelerationRPSS;

    public HoodIOKrakenx44(InternalMotorHardware pHardware) {
        this.mHoodMotor = new TalonFX(pHardware.motorID(), pHardware.canBus());

        TalonFXConfiguration HoodConfig = new TalonFXConfiguration();

        HoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        HoodConfig.CurrentLimits.SupplyCurrentLimit = pHardware.supplyCurrentLimit();
        HoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        HoodConfig.CurrentLimits.StatorCurrentLimit = pHardware.statorCurrentLimit();

        HoodConfig.MotorOutput.NeutralMode = pHardware.neutralMode();
        HoodConfig.MotorOutput.Inverted = pHardware.direction();

        HoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        HoodConfig.Feedback.SensorToMechanismRatio = pHardware.rotorToMechanismRatio();

        mHoodMotor.getConfigurator().apply(HoodConfig);

        mHoodControlMode = mHoodMotor.getControlMode();
        mHoodVelocityRPS = mHoodMotor.getVelocity();
        mHoodAccelerationRPSS = mHoodMotor.getAcceleration();
        mHoodVoltage = mHoodMotor.getMotorVoltage();
        mHoodSupplyCurrent = mHoodMotor.getSupplyCurrent();
        mHoodStatorCurrent = mHoodMotor.getStatorCurrent();
        mHoodTempCelsius = mHoodMotor.getDeviceTemp();
    }


    @Override
    public void updateInputs(HoodInputs pInputs) {
        pInputs.iIsHoodConnected = BaseStatusSignal.refreshAll(
            mHoodControlMode,
            mHoodVelocityRPS,
            mHoodAccelerationRPSS,
            mHoodVoltage,
            mHoodSupplyCurrent,
            mHoodStatorCurrent,
            mHoodTempCelsius
        ).isOK();
        pInputs.iHoodControlMode = mHoodControlMode.getValue().toString();
        pInputs.iHoodVelocityRPS = mHoodVelocityRPS.getValueAsDouble();
        pInputs.iHoodAccelerationRPSS = mHoodAccelerationRPSS.getValueAsDouble();
        pInputs.iHoodMotorVolts = mHoodVoltage.getValueAsDouble();
        pInputs.iHoodSupplyCurrentAmps = mHoodSupplyCurrent.getValueAsDouble();
        pInputs.iHoodStatorCurrentAmps = mHoodStatorCurrent.getValueAsDouble();
        pInputs.iHoodTempCelsius = mHoodTempCelsius.getValueAsDouble();
    }

    @Override
    public void setMotorPosition(double pPosition, double pFeedforward) {
        mHoodMotor.setControl(mHoodPositionControl.withPosition(pPosition).withFeedForward(pFeedforward));
    }

    @Override
    public void setMotorVolts(double pVolts) {
        mHoodMotor.setControl(mHoodVoltageControl.withOutput(pVolts));
    }

    @Override
    public void stopMotor() {
        mHoodMotor.stopMotor();
    }
}