// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final IntakeIO mIntakeIO;
  private final IntakeInputsAutoLogged mIntakeInputs = new IntakeInputsAutoLogged();
  /** Creates a new Climb. */
  public Intake(IntakeIO pIntakeIO) {
    this.mIntakeIO = pIntakeIO;
  }

  public void setIntakeVolts(double pVolts) {
    mIntakeIO.setMotorVolts(pVolts);
  }

  public void stopIntakeMotor() {
    mIntakeIO.stopMotor();
  }
  @Override
  public void periodic() {
    mIntakeIO.updateInputs(mIntakeInputs);
    Logger.processInputs("Intake", mIntakeInputs);
    // This method will be called once per scheduler run
  }
}
