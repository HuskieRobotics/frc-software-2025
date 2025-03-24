package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface ManipulatorIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ManipulatorIOInputs {

    boolean isFunnelIRBlocked = false;
    boolean isIndexerIRBlocked = false;
    boolean isAlgaeIRBlocked = false;

    boolean funnelConnected = false;
    boolean indexerConnected = false;
    boolean pivotConnected = false;

    double funnelStatorCurrentAmps = 0;
    double indexerStatorCurrentAmps = 0;
    double pivotStatorCurrentAmps = 0;

    double funnelSupplyCurrentAmps = 0;
    double indexerSupplyCurrentAmps = 0;
    double pivotSupplyCurrentAmps = 0;

    double funnelVelocityRPS = 0;
    double indexerVelocityRPS = 0;

    double funnelReferenceVelocityRPS = 0;
    double indexerReferenceVelocityRPS = 0;
    double pivotReferencePositionRot = 0;

    double indexerPositionRotations = 0.0;

    double funnelClosedLoopErrorRPS = 0.0;
    double indexerClosedLoopErrorRPS = 0.0;
    double pivotClosedLoopErrorRot = 0.0;

    double funnelTempCelsius = 0;
    double indexerTempCelsius = 0;
    double pivotTempCelsius = 0;

    double funnelMotorVoltage = 0;
    double indexerMotorVoltage = 0;
    double pivotMotorVoltage = 0;

    double pivotMotorAngleDeg = 0;

    double rightServoPosition = 0.0;
    double leftServoPosition = 0.0;
  }

  public default void updateInputs(ManipulatorIOInputs inputs) {}

  public default void setFunnelMotorVoltage(double volts) {}

  public default void setFunnelMotorVelocity(double velocity) {}

  public default void setIndexerMotorVoltage(double volts) {}

  public default void setIndexerMotorVelocity(double velocity) {}

  public default void setFunnelMotorCurrent(double current) {}

  public default void setIndexerMotorCurrent(double current) {}

  public default void setPivotMotorCurrent(double current) {}

  public default void unlockServos() {}

  public default void lockServos() {}
}
