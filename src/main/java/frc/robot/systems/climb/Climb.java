// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.climb;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;

public class Climb extends SubsystemBase {
  public enum ClimbGoal {
    kStow(() -> Units.inchesToMeters(60.0)),
    kLc1(() -> Units.inchesToMeters(0)),
    kLc2(() -> Units.inchesToMeters(0)),
    kLc3(() -> Units.inchesToMeters(0)),
    /** Custom setpoint that can be modified over network tables; Useful for debugging */
    custom(new LoggedTunableNumber("Climb/Custom", 0.0));

    private DoubleSupplier goalMeters;

    ClimbGoal(DoubleSupplier goalMeters) {
      this.goalMeters = goalMeters;
    }

    public double getGoalMeters() {
      return this.goalMeters.getAsDouble();
    }
  }

  private final ClimbIO mClimbIO;
  private final ClimbInputsAutoLogged mClimbInputs = new ClimbInputsAutoLogged();
  private ClimbGoal mCurrentGoal = null;
  private double mCurrentClimbGoalPositionMeters = 0.0;

  public Climb(ClimbIO pClimbIO) {
    this.mClimbIO = pClimbIO;
  }
  
  @Override
  public void periodic() {
    mClimbIO.updateInputs(mClimbInputs);
    Logger.processInputs("Climb", mClimbInputs);

    if (DriverStation.isDisabled()){
      stopClimbMotor();
    }

    if (mCurrentGoal != null) {
      mCurrentClimbGoalPositionMeters = mCurrentGoal.getGoalMeters();

      setPosition(mCurrentClimbGoalPositionMeters);
    }
  }

  public void setClimbVolts(double pVolts) {
    mClimbIO.setMotorVolts(pVolts);
  }

  public void stopClimbMotor() {
    mClimbIO.stopMotor();
  }

  public void setGoal(ClimbGoal pGoal){
    mCurrentGoal = pGoal;
  }

  public void setVoltage(){}

  public void setPosition(double meters){}

  @AutoLogOutput(key = "Climb/Goal")
  public ClimbGoal getGoal(){
    return mCurrentGoal;
  }



}
