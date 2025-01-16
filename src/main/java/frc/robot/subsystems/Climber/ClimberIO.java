import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    // logging values
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  // determined hardware methods
}
