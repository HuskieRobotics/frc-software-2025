package frc.robot.subsystems.intake;

public class IntakeConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private IntakeConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean TESTING = false;

  public static final int INTAKE_ROLLER_MOTOR_ID = 17;
  public static final int INTAKE_PIVOT_MOTOR_ID = 18;

  public static final double INTAKE_ROLLER_MOTORS_KP = 0.0;
  public static final double INTAKE_ROLLER_MOTORS_KS = 0.0;
  public static final double INTAKE_ROLLER_MOTORS_KV = 0.0;
  public static final double ROLLER_GEAR_RATIO = 0.0; // FIXME find the gear ratio


  public static final double INTAKE_PIVOT_MOTORS_KP = 0.0;
  public static final double INTAKE_PIVOT_MOTORS_KI = 0.0;
  public static final double INTAKE_PIVOT_MOTORS_KD = 0.0;

  public static final double INTAKE_PIVOT_MOTORS_KS = 0.0;
  public static final double INTAKE_PIVOT_MOTORS_KV = 0.0;
  public static final double INTAKE_PIVOT_MOTORS_KA = 0.0;
  public static final double INTAKE_PIVOT_MOTORS_KG = 0.0;
  public static final double PIVOT_GEAR_RATIO = 0.0; // FIXME find the gear ratio
  public static final double PIVOT_BELT_RATIO = 0.0; // FIXME find the belt ratio

  public static final double INTAKE_POSITION_PIVOT_DEGREES = 0.0;
  public static final double CARPET_POSITION_PIVOT_DEGREES = 0.0;

  public static final double INTAKE_VELOCITY_ROLLERS_RPS = 0.0;
  public static final double OUTTAKE_VELOCITY_ROLLERS_RPS = 0.0;

  public static final double ROLLER_VELOCITY_TOLERANCE = 0.0;

  public static final double ROLLERS_CONTINUOUS_SUPPLY_CURRENT_LIMIT = 0.0;
  public static final double ROLLERS_PEAK_SUPPLY_CURRENT_LIMIT = 0.0;
  public static final double ROLLERS_PEAK_SUPPLY_CURRENT_DURATION = 0.0;
  public static final double ROLLERS_ALGAE_CURRENT_LIMIT = 0.0;

  public static final double ROLLERS_CONTINUOUS_STATOR_CURRENT_LIMIT = 0.0;

  public static final double IN_BETWEEN_TIMEOUT_SECONDS = 0.0;
  public static final boolean INTAKE_PIVOT_MOTORS_INVERTED = false;
  public static final boolean INTAKE_ROLLER_MOTORS_INVERTED = false;



  public static final double PIVOT_SIM_LENGTH = 0.0;
  public static final double PIVOT_SIM_MASS = 0.0;
  public static final double PIVOT_SIM_MIN_ANGLE = 0.0;
  public static final double PIVOT_SIM_MAX_ANGLE = 0.0;
  public static final double PIVOT_SIM_STARTING_ANGLE = 0.0;

}
