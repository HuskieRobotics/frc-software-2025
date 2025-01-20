package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;

/** Generic subsystem hardware interface. */
public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {

    double voltageSuppliedLead = 0.0;
    double voltageSuppliedFollower = 0.0;
    
    double statorCurrentAmpsLead = 0.0;
    double statorCurrentAmpsFollower = 0.0;


    double supplyCurrentAmpsLead = 0.0;
    double supplyCurrentAmpsFollower = 0.0;

    double closedLoopError = 0.0; 

    double closedLoopReference = 0.0;

    double positionInches = 0.0;

    double positionRotations = 0.0;

    double LeadTempCelsius = 0.0;
    double FollowerTempCelsius = 0.0;

  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   */

  public default void setPosition(Distance position) {}

  public default void zeroPosition() {}

  public default void setVoltage(double volts) {}
}
