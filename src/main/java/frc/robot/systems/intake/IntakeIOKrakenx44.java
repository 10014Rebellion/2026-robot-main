package frc.robot.systems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;

public class IntakeIOKrakenx44 implements IntakeIO{
    private final TalonFX mIntakeMotor;
    private final VoltageOut mIntakeVoltageControl = new VoltageOut(0.0);
    
    private final StatusSignal<AngularVelocity> mIntakeVelocityMPS;
    private final StatusSignal<Voltage> mIntakeVoltage;
    private final StatusSignal<Current> mIntakeSupplyCurrent;
    private final StatusSignal<Current> mIntakeStatorCurrent;
    private final StatusSignal<Temperature> mIntakeTempCelsius;
    private final StatusSignal<AngularAcceleration> mIntakeAccelerationMPSS;
    
    public IntakeIOKrakenx44(BasicMotorHardware pConfig) {
        mIntakeMotor = new TalonFX(pConfig.motorID(), pConfig.canBus());
        var IntakeConfig = new TalonFXConfiguration();

        IntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.SupplyCurrentLimit = pConfig.supplyCurrentLimit();
        IntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.StatorCurrentLimit = pConfig.statorCurrentLimit();

        IntakeConfig.MotorOutput.NeutralMode = pConfig.neutralMode();
        IntakeConfig.MotorOutput.Inverted = pConfig.direction();

        IntakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        IntakeConfig.Feedback.SensorToMechanismRatio = pConfig.rotorToMechanismRatio();

        mIntakeVelocityMPS = mIntakeMotor.getVelocity();
        mIntakeAccelerationMPSS = mIntakeMotor.getAcceleration();
        mIntakeVoltage = mIntakeMotor.getMotorVoltage();
        mIntakeSupplyCurrent = mIntakeMotor.getSupplyCurrent();
        mIntakeStatorCurrent = mIntakeMotor.getStatorCurrent();
        mIntakeTempCelsius = mIntakeMotor.getDeviceTemp();
        
        mIntakeMotor.getConfigurator().apply(IntakeConfig);
    }

    @Override
    public void updateInputs(IntakeInputs pInputs) {
        pInputs.iIsIntakeConnected = BaseStatusSignal.refreshAll(
            mIntakeVelocityMPS,
            mIntakeAccelerationMPSS,
            mIntakeVoltage,
            mIntakeSupplyCurrent,
            mIntakeStatorCurrent,
            mIntakeTempCelsius

        ).isOK();
        pInputs.iIntakeVelocityMPS = mIntakeVelocityMPS.getValueAsDouble();
        pInputs.iIntakeAccelerationMPSS = mIntakeAccelerationMPSS.getValueAsDouble();
        pInputs.iIntakeMotorVolts = mIntakeVoltage.getValueAsDouble();
        pInputs.iIntakeSupplyCurrentAmps = mIntakeSupplyCurrent.getValueAsDouble();
        pInputs.iIntakeStatorCurrentAmps = mIntakeStatorCurrent.getValueAsDouble();
        pInputs.iIntakeTempCelsius = mIntakeTempCelsius.getValueAsDouble();
    }

    @Override
    public void setMotorVolts(double pVolts) {
        mIntakeMotor.setControl(mIntakeVoltageControl.withOutput(pVolts));
    }

    @Override
    public void stopMotor() {
        mIntakeMotor.stopMotor();
    }

}