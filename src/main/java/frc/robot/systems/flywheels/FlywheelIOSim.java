package frc.robot.systems.flywheels;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIOSim implements FlywheelIO{
    private DCMotorSim mFlywheelTopMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 4.29 / 1.0),
            DCMotor.getKrakenX60Foc(1),
            0.0,
            0.0);

    private DCMotorSim mFlywheelBottomMotor =
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 4.29 / 1.0),
        DCMotor.getKrakenX60Foc(1),
        0.0,
        0.0);
    
    
    private double mTopMotorVolts = 0.0;
    private double mBottomMotorVolts = 0.0;

    /*NOT SURE WHAT TO PUT IN HERE */
    public FlywheelIOSim(){

    }

    /*TODO: put more logger variables to in here */
    @Override
    public void updateInputs(FlywheelInputs inputs){
        mFlywheelTopMotor.update(0.02);
        mFlywheelBottomMotor.update(0.02);

        inputs.iFlywheelTopMotorVolts = mTopMotorVolts;
        inputs.iFlywheelBottomMotorVolts = mBottomMotorVolts;

    }
    @Override
    public void setTopFlywheeVolts(double volts) {
        mFlywheelTopMotor.setInputVoltage(volts);
    }

    //TODO: do flywheel PID's later
    @Override
    public void setTopFlywheePID(double kP, double kI, double kD) {}
    
    @Override
    public void setBottomFlywheeVolts(double volts) {
        mFlywheelBottomMotor.setInputVoltage(volts);
    }

    //TODO: do flywheel PID's later
    @Override
    public void setBottomFlywheePID(double kP, double kI, double kD) {}
}
