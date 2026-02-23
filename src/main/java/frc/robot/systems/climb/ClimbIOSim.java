package frc.robot.systems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.simulation.SimulationRecords.SimulatedElevator;

public class ClimbIOSim implements ClimbIO{

    private double mAppliedVolts;
    private final ElevatorSim mElevatorSim;


    public ClimbIOSim(SimulatedElevator pConfig, BasicMotorHardware pHardware) {

        mElevatorSim = new ElevatorSim(
            pConfig.kMotor(),
            pHardware.rotorToMechanismRatio(), 
            pConfig.kCarriageMassKg(), 
            pConfig.kDrumRadiusMeters(), 
            pConfig.kMinHeightMeters(),
            pConfig.kMaxHeightMeters(),
            pConfig.kSimulateGravity(), 
            pConfig.kStartingHeightMeters(), 
            pConfig.kMeasurementStdDevs());

        mAppliedVolts = 0.0;
    }

    @Override
    public void updateInputs(ClimbInputs pInputs){
        mElevatorSim.update(0.02);
        pInputs.iIsClimbConnected = true;
        pInputs.iClimbVelocityMPS = mElevatorSim.getVelocityMetersPerSecond();
        pInputs.iClimbAccelerationMPSS = 0.0;
        pInputs.iClimbMotorVolts = mAppliedVolts;
        pInputs.iClimbSupplyCurrentAmps = 0.0;
        pInputs.iClimbStatorCurrentAmps = Math.abs(mElevatorSim.getCurrentDrawAmps());
        pInputs.iClimbTempCelsius = 0.0;
        pInputs.iClimbPositionMeters = mElevatorSim.getPositionMeters();
    }

    @Override
    public void enforceSoftLimits(){
        
    }

    @Override
    public void setMotorVolts(double pVolts){
        mAppliedVolts = MathUtil.clamp(12.0, -12.0, pVolts);
        mElevatorSim.setInputVoltage(mAppliedVolts);
    }

    @Override
    public void stopMotor(){
        setMotorVolts(0.0);
    }
}
