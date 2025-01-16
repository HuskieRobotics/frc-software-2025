package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {

    double voltageSupplied = 0.0;
    double statorCurrentAmps = 0.0;
    double supplyCurrentAmps = 0.0;
    double closedLoopError = 0.0;
    double closedLoopReference = 0.0;
    double posInches = 0.0;

    double tempCelcius = 0.0;

  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   */

  public default void setMotorPosition(double position) {}


  public default void setPositionToZero() {}

  public default void setMotorVoltage(double volts) {}


}
