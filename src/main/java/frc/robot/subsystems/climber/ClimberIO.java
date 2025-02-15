package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    // logging values
    double voltage = 0.0;
    double statorCurrentAmps = 0.0;
    double supplyCurrentAmps = 0.0;
    double tempCelcius = 0.0;
    double positionRotations = 0.0;
    double positionInches = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void zeroPosition() {}
  // determined hardware methods
}
