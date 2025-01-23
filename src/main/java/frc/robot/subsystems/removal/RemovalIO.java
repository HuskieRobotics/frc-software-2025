package frc.robot.subsystems.removal;

import org.littletonrobotics.junction.AutoLog;

public interface RemovalIO {
    @AutoLog
    public static class RemovalIOInputs {
        double rollerMotorVoltage = 0;
        double rollerMotor2Voltage = 0;
        double rollerContinuousStatorCurrentLimit = 0;
        double roller2ContinuousStatorCurrentLimit = 0;
        double rollerContinuousSupplyCurrentLimit = 0;
        double roller2ContinuousSupplyCurrentLimit = 0;
        double rollerTempCelsius = 0;
        double roller2TempCelsius = 0;

    }

    public default void updateInputs(RemovalIOInputs inputs) {}

    // Roller IOMethods
    public default void setRollerMotorVoltage(double voltage) {}
    
}