package frc.robot.subsystems.subsystem.manipulator.manipulator;
import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface ManipulatorIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ManipulatorIOInputs {

    //All of the logged tunable inputs from 2024 intake code
    double positionDeg = 0.0;
    double closedLoopError = 0.0;
    double closedLoopReference = 0.0;
    double setpoint = 0.0;
    double power = 0.0;
    String controlMode = "";


    //booleans to keep track of the state of each IR sensor
    boolean isFunnelIRBlocked = false;
    boolean isIndexerIRBlocked = false;

    //not sure what these values are for but just copied them from the 2024 Intak.IO code
    //add status signals in talon fx class for all of these except for booleans
    double statorCurrentAmpsFunnel = 0;
    double statorCurrentAmpsIndexer = 0;

    double supplyCurrentAmpsFunnel = 0;
    double supplyCurrentAmpsIndexer = 0;

    double velocityRPSFunnel = 0;
    double velocityRPSIndexer = 0;

    double referenceVelocityRPSFunnel = 0;
    double referenceVelocityRPSIndexer = 0;

    double funnelClosedLoopError = 0.0;
    double indexerClosedLoopError = 0.0;

    double funnelClosedLoopReference = 0.0;
    double indexerClosedLoopReference = 0.0;

    double tempCelsiusFunnel = 0;
    double tempCelsiusIndexer = 0;

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

//I had originally created this method to check for a current spike, but I'm using the Linear Filter class for that, so I'll just keep this method here.
  /**
   * Get the current motor indexer current value specified in amps.
   * @return the current value as a double. 
   */
  public default void getIndexerMotorCurrent() {}


  /**
   * Set the funnel motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  public default void setFunnelMotorPosition(double position, double arbitraryFeedForward) {}

}


