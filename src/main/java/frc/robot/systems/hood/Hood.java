
package frc.robot.systems.hood;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private final HoodIO mHoodIO;
  private final PositionVoltage mPositionVoltageControl = new PositionVoltage(0.0);
  private final HoodIOInputsAutoLogged mHoodInputs = new HoodIOInputsAutoLogged();
  private SimpleMotorFeedforward mSimpleMotorFeedForward;
  
    public Hood(HoodIO pHoodIO, SimpleMotorFeedforward pSimpleMotorFeedforward ) {
      this.mHoodIO = pHoodIO;
      this.mSimpleMotorFeedForward = pSimpleMotorFeedforward;
  }

  public void setHoodVolts(double pVolts) {
    mHoodIO.setVoltage(pVolts);
  }

  public void stopHoodMotor() {
    mHoodIO.stop();
  }

  public void setMotorPosition(double meters){
    mHoodIO.setMotorPosition(meters, mSimpleMotorFeedForward.calculate(mHoodInputs.iClimbVelocityMPS, mHoodInputs.iClimbAccelerationMPSS));

  }

  @Override
  public void periodic() {
    mHoodIO.updateInputs(mHoodInputs);
    Logger.processInputs("Hood", mHoodInputs);
  }
}