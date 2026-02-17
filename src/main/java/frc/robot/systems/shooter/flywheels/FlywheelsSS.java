package frc.robot.systems.shooter.flywheels;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

import static frc.robot.systems.shooter.ShooterConstants.FlywheelConstants.kFlywheelControlConfig;

import java.util.function.DoubleSupplier;

public class FlywheelsSS extends SubsystemBase {

  public static enum FlywheelState{
    IDLE,
    STOP,
    VOLTAGE,
    TORQUE_CURRENT,
    TORQUE_CURRENT_BANG_BANG
  }

  private FlywheelState mCurrentState = FlywheelState.IDLE;
  
    private final FlywheelIO mLeaderFlywheelIO;
    private final FlywheelIO mFollowerFlywheelIO;
  
    private boolean mAtGoal;
  
    private final FlywheelInputsAutoLogged mLeaderFlywheelInputs = new FlywheelInputsAutoLogged();
    private final FlywheelInputsAutoLogged mFollowerFlywheelInputs = new FlywheelInputsAutoLogged();
  
    private final LoggedTunableNumber tFlywheelKP = new LoggedTunableNumber("Flywheel/Control/kP", kFlywheelControlConfig.pdController().kP());
    private final LoggedTunableNumber tFlywheelKD = new LoggedTunableNumber("Flywheel/Control/kD", kFlywheelControlConfig.pdController().kD());
  
    private final LoggedTunableNumber tFlywheelKS = new LoggedTunableNumber("Flywheel/Control/kS", kFlywheelControlConfig.feedforward().getKs());
    private final LoggedTunableNumber tFlywheelKV = new LoggedTunableNumber("Flywheel/Control/kV", kFlywheelControlConfig.feedforward().getKv());
    private final LoggedTunableNumber tFlywheelKA = new LoggedTunableNumber("Flywheel/Control/kA", kFlywheelControlConfig.feedforward().getKa());
    
    private final LoggedTunableNumber tFlywheelCruiseVel = new LoggedTunableNumber("Flywheel/Control/CruiseVel", kFlywheelControlConfig.motionMagicConstants().maxVelocity());
    private final LoggedTunableNumber tFlywheelMaxAccel = new LoggedTunableNumber("Flywheel/Control/MaxAcceleration", kFlywheelControlConfig.motionMagicConstants().maxAcceleration());
    private final LoggedTunableNumber tFlywheelMaxJerk = new LoggedTunableNumber("Flywheel/Control/MaxJerk", kFlywheelControlConfig.motionMagicConstants().maxJerk());
  
    private SimpleMotorFeedforward mFlywheelFeedforward = kFlywheelControlConfig.feedforward();
  
    public FlywheelsSS(FlywheelIO pLeaderFlywheelIO, FlywheelIO pFollowerFlywheelIO) {
      this.mLeaderFlywheelIO = pLeaderFlywheelIO;
      this.mFollowerFlywheelIO = pFollowerFlywheelIO;
    }
    
    @Override
    public void periodic() {
      mLeaderFlywheelIO.updateInputs(mLeaderFlywheelInputs);
      mFollowerFlywheelIO.updateInputs(mFollowerFlywheelInputs);
      
      refreshTuneables();
  
      if(Mode.REAL == Constants.kCurrentMode){      
        mFollowerFlywheelIO.enforceFollower(); // Sometimes during enable and disable it no longer follows briefly, this scares me, so this is a failsafe - Tnil
      }
  
      // boolean inTolerance = Math.abs(getFlywheelRPS() - mVelocityGoal) <= 5;
      // mAtGoal = atGoalDebounce.calculate(inTolerance);
      
      // Logger.recordOutput("Flywheel/inTolerance", inTolerance);
      // Logger.recordOutput("Flywheel/atGoal", mAtGoal);
  
      Logger.processInputs("Flywheel/Leader", mLeaderFlywheelInputs);
      Logger.processInputs("Flywheel/Follower", mFollowerFlywheelInputs);    
  }

  public boolean atGoal(){
    return mAtGoal;
  }  
  
  public void setFlywheelVolts(double pVolts) {  
    if(Constants.kCurrentMode == Mode.REAL){      
      mLeaderFlywheelIO.setMotorVolts(pVolts);
      mFollowerFlywheelIO.enforceFollower();
    }
    
    else{
      mLeaderFlywheelIO.setMotorVolts(pVolts);
      mFollowerFlywheelIO.setMotorVolts(pVolts); 
    }
  }

  public double getFlywheelRPS() {
    return (mLeaderFlywheelInputs.iFlywheelVelocityRPS + mFollowerFlywheelInputs.iFlywheelVelocityRPS) / 2.0;
  }

  public void setFlywheelTorqueCurrent(double pRPS) {
    if(Constants.kCurrentMode == Mode.REAL){ 
      mLeaderFlywheelIO.setMotorVelAndAccel(pRPS, 0, mFlywheelFeedforward.calculateWithVelocities(getFlywheelRPS(), pRPS));
      mFollowerFlywheelIO.enforceFollower();
    }

    else{
      mLeaderFlywheelIO.setMotorVelAndAccel(pRPS, 0, mFlywheelFeedforward.calculateWithVelocities(getFlywheelRPS(), pRPS));
      mFollowerFlywheelIO.setMotorVelAndAccel(pRPS, 0, mFlywheelFeedforward.calculateWithVelocities(getFlywheelRPS(), pRPS));
    }
  }

  public void stopFlywheelMotors() {

    if(Constants.kCurrentMode == Mode.REAL){
      mLeaderFlywheelIO.stopMotor();
      mFollowerFlywheelIO.enforceFollower();
    }

    else{
      mLeaderFlywheelIO.stopMotor();
      mFollowerFlywheelIO.stopMotor();
    }
  }

  private void setBothPDConstants(double pKP, double pKD) {
    mLeaderFlywheelIO.setPDConstants(pKP, pKD);
    mFollowerFlywheelIO.setPDConstants(pKP, pKD);
  }

  private void setBothMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {
    mLeaderFlywheelIO.setMotionMagicConstants(pCruiseVel, pMaxAccel, pMaxJerk);
    mFollowerFlywheelIO.setMotionMagicConstants(pCruiseVel, pMaxAccel, pMaxJerk);
  }
  

  private void refreshTuneables() {
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setBothPDConstants(tFlywheelKP.get(), tFlywheelKD.get()), 
      tFlywheelKP, tFlywheelKD
    );
  
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setBothMotionMagicConstants(tFlywheelCruiseVel.get(), tFlywheelMaxAccel.get(), tFlywheelMaxJerk.get()), 
      tFlywheelCruiseVel, tFlywheelMaxAccel, tFlywheelMaxJerk
    );

    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> mFlywheelFeedforward = new SimpleMotorFeedforward(tFlywheelKS.get(), tFlywheelKV.get(), tFlywheelKA.get()),
      tFlywheelKS, tFlywheelKV, tFlywheelKA
    );
  }
}
