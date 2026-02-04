// REBELLION 10014

package frc.robot.systems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.Records.InternalMotorHardware;

public class HoodIOKrakenx44 implements HoodIO {
    private final TalonFX mHoodMotor;
    private final VoltageOut mHoodVoltageControl = new VoltageOut(0.0);
    private final PositionVoltage mPositionVoltageControl = new PositionVoltage(0.0);

    private final StatusSignal<AngularVelocity> mHoodVelocityMPS;
    private final StatusSignal<Voltage> mHoodVoltage;
    private final StatusSignal<Current> mHoodSupplyCurrent;
    private final StatusSignal<Current> mHoodStatorCurrent;
    private final StatusSignal<Temperature> mHoodTempCelsius;
    private final StatusSignal<AngularAcceleration> mHoodAccelerationMPSS;


    private double kS = 0.0;
    private double kV = 0.0;
    private double kA = 0.0;
    private double kG = 0.0;

    public HoodIOKrakenx44(InternalMotorHardware pConfig) {
        mHoodMotor = new TalonFX(pConfig.motorID(), pConfig.canBus());
        var HoodConfig = new TalonFXConfiguration();

        HoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        HoodConfig.CurrentLimits.SupplyCurrentLimit = pConfig.supplyCurrentLimit();
        HoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        HoodConfig.CurrentLimits.StatorCurrentLimit = pConfig.statorCurrentLimit();

        HoodConfig.Voltage.PeakForwardVoltage = 12;
        HoodConfig.Voltage.PeakReverseVoltage = -12;

        HoodConfig.MotorOutput.NeutralMode = pConfig.neutralMode();
        HoodConfig.MotorOutput.Inverted = pConfig.direction();

        HoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        HoodConfig.Feedback.SensorToMechanismRatio = pConfig.rotorToMechanismRatio();

        mHoodVelocityMPS = mHoodMotor.getVelocity();
        mHoodAccelerationMPSS = mHoodMotor.getAcceleration();
        mHoodVoltage = mHoodMotor.getMotorVoltage();
        mHoodSupplyCurrent = mHoodMotor.getSupplyCurrent();
        mHoodStatorCurrent = mHoodMotor.getStatorCurrent();
        mHoodTempCelsius = mHoodMotor.getDeviceTemp();

        mHoodMotor.getConfigurator().apply(HoodConfig);
    }

    @Override
    public void updateInputs(HoodIOInputs pInputs) {
        pInputs.iIsHoodConnected = BaseStatusSignal.refreshAll(
            mHoodVelocityMPS,
            mHoodAccelerationMPSS,
            mHoodVoltage,
            mHoodSupplyCurrent,
            mHoodStatorCurrent,
            mHoodTempCelsius
        ).isOK();
        pInputs.iHoodVelocityMPS = mHoodVelocityMPS.getValueAsDouble();
        pInputs.iHoodAccelerationMPSS = mHoodAccelerationMPSS.getValueAsDouble();
        pInputs.iHoodMotorVolts = mHoodVoltage.getValueAsDouble();
        pInputs.iHoodSupplyCurrentAmps = mHoodSupplyCurrent.getValueAsDouble();
        pInputs.iHoodStatorCurrentAmps = mHoodStatorCurrent.getValueAsDouble();
        pInputs.iHoodTempCelsius = mHoodTempCelsius.getValueAsDouble();
    }

    @Override
    public void setVoltage(double pVolts) {
        mHoodMotor.setControl(mHoodVoltageControl.withOutput(pVolts));
    }

    @Override
    public void stop() {
        mHoodMotor.stopMotor();
    }

    @Override
    public void setMotorPosition(double pPositionM, double pFeedforward) {
        mHoodMotor.setControl(mPositionVoltageControl.withPosition(pPositionM).withFeedForward(pFeedforward));
    }


    @Override
    public void setDrivePID(int pSlot, double pKP, double pKI, double pKD) {
        var slotConfig = new SlotConfigs();
        slotConfig.SlotNumber = pSlot;
        slotConfig.kP = pKP;
        slotConfig.kI = pKI;
        slotConfig.kD = pKD;
    }

    @Override
    public void setFeedforward(double kS, double kV, double kA, double kG) { //this is to set the feedworward
        this.kS = kS;
        this.kV = kV;
        this.kA = kA; //dont know this
        this.kG = kG;
    }
}