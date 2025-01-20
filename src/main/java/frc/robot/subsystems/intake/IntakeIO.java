package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        double rollerMotorVoltage = 0;
        double rollerMotorVelocityRPS = 0;
        double rollerStatorCurrentAmps = 0;
        double rollerSupplyCurrentAmps = 0;
        double rollerTempCelcius = 0;
        double rollerClosedLoopError = 0;
        double rollerReferenceVelocityRPS = 0;

        double pivotMotorVoltage = 0;
        double pivotPositionDeg = 0;
        double pivotStatorCurrentAmps = 0;
        double pivotSupplyCurrentAmps = 0;
        double pivotTempCelcius = 0;
        double pivotClosedLoopError = 0;
        double pivotReferencePositionDeg = 0;
    }
// pivot IOMethods
    public default void updateInputs(IntakeIOInputs inputs){}
    
    public default void setPivotRotationPosition(double position){}

    public default void setPivotVoltage(double voltage){}


// roller IOMethods
    
    public default void setRollerMotorVoltage(double voltage){}

    public default void setRollerMotorVelocity(double velocity){}
}


