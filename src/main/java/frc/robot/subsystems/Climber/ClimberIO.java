import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    // logging values
    double voltage = 0.0;
    double staterCurrentAmps = 0.0;
    double supplyCurrentAmps = 0.0;
    double tempCelcius = 0.0;
    double position_units_per_rotation = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void zeroPosition() {}
  // determined hardware methods
}
