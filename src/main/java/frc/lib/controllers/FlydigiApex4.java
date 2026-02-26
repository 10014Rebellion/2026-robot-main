package frc.lib.controllers;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FlydigiApex4 extends CommandXboxController {
    private final int mPort;

    public FlydigiApex4(int pDriverStationPort) {
        super(pDriverStationPort);
        mPort = pDriverStationPort;
    }

    public Trigger selectButton() {
        return super.button(7);
    }

    public Trigger startButton() {
        return super.button(8);
    }

    public Rotation2d getPOVAngle() {
        double povAngle = super.getHID().getPOV();
        if(povAngle == -1) DriverStation.reportError("<<< FATAL: NO POV WAS DETECTED ON CONTROLLER PORT: " + mPort + " >>>", true);
        return Rotation2d.fromDegrees(povAngle);
    }
    
    public Supplier<Rotation2d> getPOVAngleSup() {
        return () -> getPOVAngle();
    }

    public DoubleSupplier getLeftXSup() {
        return () -> super.getLeftX();
    }

    public DoubleSupplier getLeftYSup() {
        return () -> super.getLeftY();
    }

    public DoubleSupplier getRightXSup() {
        return () -> super.getRightX();
    }

    public DoubleSupplier getRightYSup() {
        return () -> super.getRightY();
    }

    public DoubleSupplier getRightTriggerSup() {
        return () -> super.getRightTriggerAxis();
    }

    public DoubleSupplier getLeftTriggerSup() {
        return () -> super.getLeftTriggerAxis();
    }
}
