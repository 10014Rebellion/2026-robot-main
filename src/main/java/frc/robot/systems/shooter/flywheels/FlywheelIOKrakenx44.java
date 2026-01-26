package frc.robot.systems.shooter.flywheels;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.hardware.HardwareRecords.InternalMotorHardware;

public class FlywheelIOKrakenx44 implements FlywheelIO{
    private final TalonFX mFlywheelMotor;
    private final VoltageOut mFlywheelVoltageControl = new VoltageOut(0.0);
    private final VelocityDutyCycle mFlywheelVelocityControl = new VelocityDutyCycle(0.0);
    private final StatusSignal<ControlModeValue> mFlywheelControlMode;
    private final StatusSignal<AngularVelocity> mFlywheelVelocityRPS;
    private final StatusSignal<Voltage> mFlywheelVoltage;
    private final StatusSignal<Current> mFlywheelSupplyCurrent;
    private final StatusSignal<Current> mFlywheelStatorCurrent;
    private final StatusSignal<Temperature> mFlywheelTempCelsius;
    private final StatusSignal<AngularAcceleration> mFlywheelAccelerationRPSS;
    private final boolean mIsLeader;

    // FOLLOWER CONSTRUCTOR
    public FlywheelIOKrakenx44(FollowerMotorHardware pFollowerConfig) {
        this(pFollowerConfig.motorID(), pFollowerConfig.leaderConfig(), false);
        mFlywheelMotor.setControl(new Follower(pFollowerConfig.leaderConfig().motorID(), pFollowerConfig.alignmentValue()));
    }
    
    // LEADER CONSTRUCTOR
    public FlywheelIOKrakenx44(InternalMotorHardware pLeaderConfig) {
        this(pLeaderConfig.motorID(), pLeaderConfig, true);
    }

    private FlywheelIOKrakenx44(int pMotorID, InternalMotorHardware pHardware, boolean pIsLeader) {
        this.mFlywheelMotor = new TalonFX(pMotorID, pHardware.canBus());
        this.mIsLeader = pIsLeader;

        TalonFXConfiguration FlywheelConfig = new TalonFXConfiguration();

        FlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        FlywheelConfig.CurrentLimits.SupplyCurrentLimit = pHardware.supplyCurrentLimit();
        FlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        FlywheelConfig.CurrentLimits.StatorCurrentLimit = pHardware.statorCurrentLimit();

        FlywheelConfig.MotorOutput.NeutralMode = pHardware.neutralMode();
        FlywheelConfig.MotorOutput.Inverted = pHardware.direction();

        FlywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        FlywheelConfig.Feedback.SensorToMechanismRatio = pHardware.rotorToMechanismRatio();

        mFlywheelMotor.getConfigurator().apply(FlywheelConfig);

        mFlywheelControlMode = mFlywheelMotor.getControlMode();
        mFlywheelVelocityRPS = mFlywheelMotor.getVelocity();
        mFlywheelAccelerationRPSS = mFlywheelMotor.getAcceleration();
        mFlywheelVoltage = mFlywheelMotor.getMotorVoltage();
        mFlywheelSupplyCurrent = mFlywheelMotor.getSupplyCurrent();
        mFlywheelStatorCurrent = mFlywheelMotor.getStatorCurrent();
        mFlywheelTempCelsius = mFlywheelMotor.getDeviceTemp();
    }

    @Override
    public void updateInputs(FlywheelInputs pInputs) {
        pInputs.iIsFlywheelConnected = BaseStatusSignal.refreshAll(
            mFlywheelControlMode,
            mFlywheelVelocityRPS,
            mFlywheelAccelerationRPSS,
            mFlywheelVoltage,
            mFlywheelSupplyCurrent,
            mFlywheelStatorCurrent,
            mFlywheelTempCelsius
        ).isOK();
        pInputs.iFlywheelControlMode = mFlywheelControlMode.getValue().toString();
        pInputs.iFlywheelVelocityRPS = mFlywheelVelocityRPS.getValueAsDouble();
        pInputs.iFlywheelAccelerationRPSS = mFlywheelAccelerationRPSS.getValueAsDouble();
        pInputs.iFlywheelMotorVolts = mFlywheelVoltage.getValueAsDouble();
        pInputs.iFlywheelSupplyCurrentAmps = mFlywheelSupplyCurrent.getValueAsDouble();
        pInputs.iFlywheelStatorCurrentAmps = mFlywheelStatorCurrent.getValueAsDouble();
        pInputs.iFlywheelTempCelsius = mFlywheelTempCelsius.getValueAsDouble();
    }

    @Override
    public void setMotorVelocity(double pVelocityRPS, double pFeedforward) {
        if(mIsLeader) mFlywheelMotor.setControl(mFlywheelVelocityControl.withVelocity(pVelocityRPS).withFeedForward(pFeedforward));
    }

    @Override
    public void setMotorVolts(double pVolts) {
        if(mIsLeader) mFlywheelMotor.setControl(mFlywheelVoltageControl.withOutput(pVolts));
    }

    @Override
    public void stopMotor() {
        if(mIsLeader) mFlywheelMotor.stopMotor();
    }
}