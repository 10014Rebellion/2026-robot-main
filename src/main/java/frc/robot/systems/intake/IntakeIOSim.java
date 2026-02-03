package frc.robot.systems.intake;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;

public class IntakeIOSim implements IntakeIO{
    
    private final DCMotorSim mIntakeSim;
    private double mAppliedVolts;

    public IntakeIOSim(BasicMotorHardware pConfig) {
        mIntakeSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.004, pConfig.rotorToMechanismRatio()),
            DCMotor.getKrakenX44(1).withReduction(pConfig.rotorToMechanismRatio()),
            0.0,
            0.0);

        mAppliedVolts = 0.0;
    }

    @Override
    public void updateInputs(IntakeInputs pInputs) {
        pInputs.iIsIntakeConnected = true;
        pInputs.iIntakeVelocityMPS = mIntakeSim.getAngularVelocityRPM() * 60.0;
        pInputs.iIntakeAccelerationMPSS = mIntakeSim.getAngularAccelerationRadPerSecSq();
        pInputs.iIntakeMotorVolts = mAppliedVolts;
        pInputs.iIntakeSupplyCurrentAmps = 0.0;
        pInputs.iIntakeStatorCurrentAmps = Math.abs(mIntakeSim.getCurrentDrawAmps());
        pInputs.iIntakeTempCelsius = 0.0;
    }

    @Override
    public void setMotorVolts(double pVolts) {
        mIntakeSim.setInputVoltage(pVolts);
        mAppliedVolts = pVolts;
    }

    @Override
    public void stopMotor() {
        setMotorVolts(0.0);
    }
}
