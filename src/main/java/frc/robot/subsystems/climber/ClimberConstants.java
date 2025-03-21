package frc.robot.subsystems.climber;

public class ClimberConstants {
  public static final int CLIMBER_MOTOR_CAN_ID = 16;

  public static final boolean CLIMBER_MOTOR_INVERTED = false; // untested, told to change
  public static final double CLIMBER_CONTINUOUS_CURRENT_LIMIT = 40.0;
  public static final double CLIMBER_STATOR_CURRENT_LIMIT = 60.0;
  public static final double CLIMBER_PEAK_CURRENT_LIMIT = 60.0;
  public static final double CLIMBER_PEAK_CURRENT_DURATION = 2.0;
  public static final double GEAR_RATIO = 25.0;

  public static final double EXTEND_VOLTAGE = -3.0;

  // FIXME: tune
  public static final double CLIMB_VOLTAGE = 3.0;

  public static final double RETRACT_VOLTAGE_SLOW = 3.0;

  public static final double DRUM_DIAMETER = 1.0;
  public static final double CAGE_CATCHER_EXTEND_POS_INCHES = 5.0; // FIXME: arbitrary for now
  public static final double MIN_HEIGHT_INCHES = 2.25; // worked at 2.0, but barely
  public static final double MAX_HEIGHT_INCHES = 15.0;
}
