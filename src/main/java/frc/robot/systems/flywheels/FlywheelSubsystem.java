package frc.robot.systems.flywheels;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase{
    private FlywheelIO io;
    private FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();

    public FlywheelSubsystem(FlywheelIO pIo) {
        this.io = pIo;
    }

    @Override
    public void periodic(){
        io.recordOutput(inputs);
    }

    public void setTopFlyWheelVolts(double pVolts){
        io.setTopFlywheeVolts(pVolts);
    }

    //TODO: acc make this method later
    public void setTopPID(double kP, double kD, double kI){
        io.setTopFlywheePID(kP, kD, kI);
    }

    
    public void setBottomFlyWheelVolts(double pVolts){
        io.setBottomFlywheeVolts(pVolts);
    }

    //TODO: acc make this method later
    public void setBottomPID(double kP, double kD, double kI){
        io.setBottomFlywheePID(kP, kD, kI);
    }

    /*TEMPORARY SHOOTER METHOD!! */
    //TODO: need to make better later!
    public Command shootCmd() {
        return new FunctionalCommand(
            () -> {
                setTopFlyWheelVolts(-12);
                setBottomFlyWheelVolts(-12);
            },
            () -> {},
            (interrupted) -> {
                setTopFlyWheelVolts(0);
                setBottomFlyWheelVolts(0);
            },
            () -> false,
            this);
    }

}
