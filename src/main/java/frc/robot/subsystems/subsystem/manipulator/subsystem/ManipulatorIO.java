package frc.robot.subsystems.subsystem.manipulator.subsystem;
import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface ManipulatorIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ManipulatorIOInputs {

    //copied from 2024 code
    double positionDeg = 0.0;
    double velocityRPM = 0.0;
    double closedLoopError = 0.0;
    double setpoint = 0.0;
    double power = 0.0;
    String controlMode = "";
    double statorCurrentAmps = 0.0;
    double tempCelsius = 0.0;
    double supplyCurrentAmps = 0.0;

    //booleans to keep track of the state of each IR sensor
    boolean isFunnelIRBlocked = false;
    boolean isIndexerIRBlocked = false;

    //not sure what these values are for but just copied them from the 2024 Intak.IO code
    double funnelMotorStatorCurrentAmps = 0;
    double indexerMotorCurrentAmps = 0;

    double funnelMotorSupplyCurrentAmps = 0;
    double indexerMotorSupplyCurrentAmps = 0;

    double funnelMotorVelocityRPS = 0;
    double indexerMotorVelocityRPS = 0;

    double funnelMotorReferenceVelocityRPS = 0;
    double indexerMotorReferenceVelocityRPS = 0;

    double funnelMotorTempCelsius = 0;
    double indexerMotorTempCelsius = 0;

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

  /**
   * Set the funnel motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  public default void setFunnelMotorPosition(double position, double arbitraryFeedForward) {}

   /**
   * Set the indexer motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  public default void setIndexerMotorPosition(double position, double arbitraryFeedForward) {}

  /**
   * Get the state of the IR sensor near the funnel motor.
   * 
   * @return true if the ir sensor near the funnel is blocked
   */
  public default boolean getFunnelIRState() {return false;} //i just put false as the default value, it will be overriden in the ManipulatorIOTalonFX.java class

  /**
   * Get the state of the IR sensor near the indexer motor.
   * 
   * @return true if the ir sensor near the indexer is blocked
   */
  public default boolean getIndexerIRState() {return false;}
}

