package frc.robot.subsystems.removal;

import org.littletonrobotics.junction.AutoLog;

public interface RemovalIO {
    @AutoLog
    public static class RemovalIOInputs {
        double rollerMotorVoltage = 0;
        double rollersContinuousStatorCurrentLimit = 0;
        double rollersContinuousSupplyCurrentLimit = 0;
        double rollerOpenLoopError = 0;
        double rollerTempCelsius = 0;

    }

    // Roller IOMethods
    public void setRollerMotorVoltage(double voltage);
    
}