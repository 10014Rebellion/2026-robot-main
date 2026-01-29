package frc.robot.systems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.ElevatorController;

public class ClimbIOSim implements ClimbIO{

    private final DCMotorSim mClimbMotor;
    // private final ElevatorSim mElevatorSim;
    private double mAppliedVolts;
    private PIDController mClimbPID;

    private final BasicMotorHardware mConfig;
    private final ElevatorController mController;


    public ClimbIOSim(BasicMotorHardware pConfig, ElevatorController pController) {

        this.mConfig = pConfig;
        this.mController = pController;

        mClimbMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44(1), 
                0.004, 
                mConfig.rotorToMechanismRatio()),
            DCMotor.getKrakenX60Foc(1),
            0.0,
            0.0
        );

        // mElevatorSim = new ElevatorSim()

        mClimbPID = new PIDController(0.0, 0.0, 0.0);

        mAppliedVolts = 0.0;
    }

    @Override
    public void updateInputs(ClimbInputs pInputs){
        mClimbMotor.update(0.02);
        pInputs.iIsClimbConnected = true;
        pInputs.iClimbVelocityMPS = (mClimbMotor.getAngularVelocityRPM() * mConfig.rotorToMechanismRatio()) / 60.0;
        pInputs.iClimbAccelerationMPSS = mClimbMotor.getAngularAccelerationRadPerSecSq() * mConfig.rotorToMechanismRatio();
        pInputs.iClimbMotorVolts = mAppliedVolts;
        pInputs.iClimbSupplyCurrentAmps = 0.0;
        pInputs.iClimbStatorCurrentAmps = Math.abs(mClimbMotor.getCurrentDrawAmps());
        pInputs.iClimbTempCelsius = 0.0;
        pInputs.iClimbPositionMeters = mClimbMotor.getAngularPositionRotations() * mConfig.rotorToMechanismRatio();
    }

    @Override
    public void setMotorVolts(double pVolts){
        mAppliedVolts = MathUtil.clamp(12.0, -12.0, pVolts);
        mClimbMotor.setInputVoltage(mAppliedVolts);
    }

    @Override
    public void setMotorPosition(double pPositionM, double pFeedforward){
        setMotorVolts(mClimbPID.calculate(mClimbMotor.getAngularPositionRad(), pPositionM) + pFeedforward);
    }

    @Override
    public void setPIDConstants(int pSlot, double pKP, double pKI, double pKD){
    }

    @Override
    public void stopMotor(){
        setMotorVolts(0.0);
    }
}
