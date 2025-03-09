package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface ManipulatorIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ManipulatorIOInputs {

    // booleans to keep track of the state of each IR sensor
    boolean isFunnelIRBlocked = false;
    boolean isIndexerIRBlocked = false;

    boolean funnelConnected = false;
    boolean indexerConnected = false;

    double funnelStatorCurrentAmps = 0;
    double indexerStatorCurrentAmps = 0;

    double funnelSupplyCurrentAmps = 0;
    double indexerSupplyCurrentAmps = 0;

    double funnelVelocityRPS = 0;
    double indexerVelocityRPS = 0;

    double funnelReferenceVelocityRPS = 0;
    double indexerReferenceVelocityRPS = 0;

    double funnelClosedLoopErrorRPS = 0.0;
    double indexerClosedLoopErrorRPS = 0.0;

    double funnelTempCelsius = 0;
    double indexerTempCelsius = 0;

    double funnelMotorVoltage = 0;
    double indexerMotorVoltage = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ManipulatorIOInputs inputs) {}

  /**
   * Set the funnel motor voltage to the specified number of volts.
   *
   * @param volts the number of volts to set the motor voltage to.
   */
  public default void setFunnelMotorVoltage(double volts) {}

  /**
   * Set the funnel motor velocity to the specified velocity.
   *
   * @param velocity the specified velocity to set the motor to.
   */
  public default void setFunnelMotorVelocity(double velocity) {}

  /**
   * Set the indexer motor voltage to the specified number of volts.
   *
   * @param volts the number of volts to set the motor voltage to.
   */
  public default void setIndexerMotorVoltage(double volts) {}

  /**
   * Set the indexer motor velocity to the specified velocity.
   *
   * @param velocity the specified velocity to set the motor to.
   */
  public default void setIndexerMotorVelocity(double velocity) {}

  /**
   * Set the funnel motor current to the specified value in amps.
   *
   * @param current the current to set the motor to in amps.
   */
  public default void setFunnelMotorCurrent(double current) {}

  /**
   * Set the indexer motor current to the specified value in amps.
   *
   * @param current the current to set the motor to in amps.
   */
  public default void setIndexerMotorCurrent(double current) {}
}
