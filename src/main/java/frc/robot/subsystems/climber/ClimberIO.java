package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    // logging values
    boolean connected = false;
    double voltage = 0.0;
    double statorCurrentAmps = 0.0;
    double supplyCurrentAmps = 0.0;
    double tempCelsius = 0.0;
    double positionRotations = 0.0;
    double positionInches = 0.0;
    boolean limitSwitch1Engaged = false;
    boolean limitSwitch2Engaged = false;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void zeroPosition() {}
}
