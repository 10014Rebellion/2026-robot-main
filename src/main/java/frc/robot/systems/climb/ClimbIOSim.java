package frc.robot.systems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO{
    private final DCMotorSim mClimbMotor;
    private double mAppliedVolts;
    

    public ClimbIOSim() {
        mClimbMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.004, 0.0),
            DCMotor.getKrakenX60Foc(1),
            0.0,
            0.0
        );

        mAppliedVolts = 0.0;
    }

    @Override
    public void updateInputs(ClimbInputs pInputs){

    }

    @Override
    public void setMotorVolts(double pVolts){
        mAppliedVolts = MathUtil.clamp(12.0, -12.0, pVolts);
        mClimbMotor.setInputVoltage(mAppliedVolts);
    }

    @Override
    public void setMotorPosition(double pPositionM, double pFeedforward){

    }

    @Override
    public void stopMotor(){

    }
}
