package frc.robot.systems.shooter.hood;

import org.littletonrobotics.junction.Logger;

public class Hood {
  private final HoodIO mHoodIO;

  private final HoodInputsAutoLogged mHoodInputs = new HoodInputsAutoLogged();

  public Hood(HoodIO pHoodIO) {
    this.mHoodIO = pHoodIO;
  }

  public void setHoodVolts(double pVolts) {
    mHoodIO.setMotorVolts(pVolts);
  }

  public void stopHoodMotor() {
    mHoodIO.stopMotor();
  }
  
  public void periodic() {
    mHoodIO.updateInputs(mHoodInputs);
    Logger.processInputs("Hood", mHoodInputs);
  }
}
