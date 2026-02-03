// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;

public class Intake extends SubsystemBase {

  public enum IntakeGoal {
    kIntake(() -> 8.0),
    kOuttake(() -> -6.0),
    kStop(() -> 0.0),
    /** Custom setpoint that can be modified over network tables; Useful for debugging */
    custom(new LoggedTunableNumber("Intake/Custom", 0.0));

    private DoubleSupplier goalMeters;

    IntakeGoal(DoubleSupplier goalMeters) {
      this.goalMeters = goalMeters;
    }

    public double getGoalVolts() {
      return this.goalMeters.getAsDouble();
    }
  }


  private final IntakeIO mIntakeIO;
  private final IntakeInputsAutoLogged mIntakeInputs = new IntakeInputsAutoLogged();

  private IntakeGoal mCurrentGoal = null;
  private double mCurrentIntakeGoalVolts = 0.0;

  /** Creates a new Climb. */
  public Intake(IntakeIO pIntakeIO) {
    this.mIntakeIO = pIntakeIO;
  }
  
  @Override
  public void periodic() {
    mIntakeIO.updateInputs(mIntakeInputs);
    Logger.processInputs("Intake", mIntakeInputs);
    
    if(DriverStation.isDisabled()){
      stopIntakeMotor();
    }

    if (mCurrentGoal != null){
      mCurrentIntakeGoalVolts = mCurrentGoal.getGoalVolts();

      setIntakeVolts(mCurrentIntakeGoalVolts);
    }

  }

  public void setIntakeVolts(double pVolts) {
    mIntakeIO.setMotorVolts(pVolts);
  }

  public void stopIntakeMotor() {
    mIntakeIO.stopMotor();
  }

    public void setGoal(IntakeGoal pGoal){
    mCurrentGoal = pGoal;
  }

  @AutoLogOutput(key = "Intake/Goal")
  public IntakeGoal getGoal(){
    return mCurrentGoal;
  }
  
}
