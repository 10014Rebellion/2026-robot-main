// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.systems.intake.pivot.IntakePivotSS;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotState;
import frc.robot.systems.intake.roller.IntakeRollerSS;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;

public class Intake {
  private final IntakePivotSS mIntakePivotSS;
  private final IntakeRollerSS mIntakeRollerSS;

  public Intake(IntakePivotSS pIntakePivotSS, IntakeRollerSS pIntakeRollerSS) {
    this.mIntakePivotSS = pIntakePivotSS;
    this.mIntakeRollerSS = pIntakeRollerSS;
  }

  public Command setRollerState(IntakeRollerState rollerState) {
    return mIntakeRollerSS.setIntakeRollerState(rollerState);
  }

  public Command setRollerVoltsManualCmd(double pVolts) {
    return mIntakeRollerSS.setIntakeVoltsManualCmd(pVolts);
  }
  
  public Command stopRollerMotorManualCmd() {
    return mIntakeRollerSS.stopIntakeVoltsManualCmd();
  }

  public Command stopPivotMotorCmd() {
    return new InstantCommand(() -> mIntakePivotSS.stopPivotMotor(), mIntakePivotSS);
  }

  public Command setPivotTuneableAmps() {
    return new InstantCommand(() -> mIntakePivotSS.setCustomPivotAmps(), mIntakePivotSS);
  }

  public Command setPivotState(IntakePivotState intakePivotState) {
    return Commands.run(() -> mIntakePivotSS.setPivotState(intakePivotState), mIntakePivotSS);
  }

  public Command setPivotVolts(double pVolts) {
    return Commands.run(() -> mIntakePivotSS.setPivotVolts(pVolts), mIntakePivotSS);
  }

  public Command setPivotRotManualCmd(Rotation2d pRotSP) {
    return Commands.run(() -> mIntakePivotSS.setPivotRotManually(pRotSP), mIntakePivotSS);
  }

  public Command holdPivotCmd(){
    return new InstantCommand(() -> mIntakePivotSS.holdPivot());
  }

  public Command setPivotRotCmd() {
    return Commands.run(() -> mIntakePivotSS.setPivotRotManually(), mIntakePivotSS);
  }
}
