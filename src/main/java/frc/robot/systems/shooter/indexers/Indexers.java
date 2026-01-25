package frc.robot.systems.shooter.indexers;

import org.littletonrobotics.junction.Logger;

public class Indexers {
  private final IndexerIO mLeaderIndexerIO;
  private final IndexerIO mFollowerIndexerIO;

  private final IndexerInputsAutoLogged mLeaderIndexerInputs = new IndexerInputsAutoLogged();
  private final IndexerInputsAutoLogged mFollowerIndexerInputs = new IndexerInputsAutoLogged();

  public Indexers(IndexerIO pLeaderIndexerIO, IndexerIO pFollowerIndexerIO) {
    this.mLeaderIndexerIO = pLeaderIndexerIO;
    this.mFollowerIndexerIO = pFollowerIndexerIO;
  }

  public void setIndexerVolts(double pVolts) {
    mLeaderIndexerIO.setMotorVolts(pVolts);
  }

  public void stopIndexerMotor() {
    mLeaderIndexerIO.stopMotor();
  }
  
  public void periodic() {
    mLeaderIndexerIO.updateInputs(mLeaderIndexerInputs);
    mFollowerIndexerIO.updateInputs(mFollowerIndexerInputs);
    Logger.processInputs("Indexer/Leader", mLeaderIndexerInputs);
    Logger.processInputs("Indexer/Follower", mFollowerIndexerInputs);
  }
}
