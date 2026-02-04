// REBELLION 10014
/* 
package frc.robot.systems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.hardware.Records.InternalMotorHardware;

public class HoodIOSim implements HoodIO {
    private DCMotorSim driveMotor = 
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, kDriveMotorGearing), 
        DCMotor.getKrakenX60Foc(1), 0.0, 0.0);

    private double hoodVolts = 0.0;
    private final TalonFX mHoodMotor;
    private final VoltageOut mHoodVoltageControl = new VoltageOut(0.0);



    public HoodIOSim() {
        
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
    public void setDrivePID(double pKP, double pKI, double pKD) {
        var slotConfig = new Slot0Configs();
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

*/