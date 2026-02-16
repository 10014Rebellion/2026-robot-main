package frc.robot.systems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.hood.HoodSS;

public class Shooter {
  private final FuelPumpSS mFuelPumpsSS;
  private final HoodSS mHoodSS;
  private final FlywheelsSS mFlywheelSS;

  private Debouncer mFlywheelDebouncer = new Debouncer(0.25, DebounceType.kRising);

  public Shooter(FuelPumpSS pFuelPumpSS, HoodSS pHoodSS, FlywheelsSS pFlywheelSS) {
    this.mFuelPumpsSS = pFuelPumpSS; this.mHoodSS = pHoodSS; this.mFlywheelSS = pFlywheelSS; // I miss my C++ initializer lists :'(
  }

  public Command setFlywheelsRPSCmd(double pRPS) {
    return new InstantCommand(() -> mFlywheelSS.setFlywheelTorqueCurrent(pRPS), mFlywheelSS);
  }

  public Command setFlywheelsRPSBangBangCommand(double pRPS) {
    return new FunctionalCommand(
      () -> {}, 
      () -> {
        boolean inTolerance = Math.abs(mFlywheelSS.getFlywheelRPS() - pRPS) <= 5;
        boolean atGoal = mFlywheelDebouncer.calculate(inTolerance);


        if(atGoal){
          mFlywheelSS.setFlywheelTorqueCurrent(40);
        }

        else{
          mFlywheelSS.setFlywheelTorqueCurrent(100);
        }
      }, 

      (interrupted) -> {mFlywheelSS.setFlywheelTorqueCurrent(30);}, 
      () -> false ,
      mFlywheelSS);
  }

  public Command setFlywheelsVoltsCmd(double pVolts) {
    return new InstantCommand(() -> mFlywheelSS.setFlywheelVolts(pVolts));
  }

  public Command setFuelPumpsVoltsCmd(double pVolts) {
    return new InstantCommand(() -> mFuelPumpsSS.setFuelPumpVolts(pVolts));
  }

  public Command setFuelPumpRPSCmd(double pRPS) {
    return new InstantCommand(() -> mFuelPumpsSS.setFuelPumpRPS(pRPS));
  }

  public Command setHoodRot(Rotation2d pRot) {
    return new InstantCommand(() -> mHoodSS.setHoodRot(pRot));
  }

  public Command holdHoodCmd() {
    return new InstantCommand(() -> mHoodSS.holdHood());
  }

  public Command setHoodVoltsCmd(double pVolts) {
    return new InstantCommand(() -> mHoodSS.setHoodVolts(pVolts));
  }
}
