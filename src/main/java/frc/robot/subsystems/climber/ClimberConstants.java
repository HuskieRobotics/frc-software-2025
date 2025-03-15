package frc.robot.subsystems.climber;

public class ClimberConstants {
  public static final int CLIMBER_MOTOR_CAN_ID = 16;

  public static final boolean CLIMBER_MOTOR_INVERTED = false; // untested, told to change
  public static final double CLIMBER_CONTINUOUS_CURRENT_LIMIT = 40.0;
  public static final double CLIMBER_STATOR_CURRENT_LIMIT = 60.0;
  public static final double CLIMBER_PEAK_CURRENT_LIMIT = 60.0;
  public static final double CLIMBER_PEAK_CURRENT_DURATION = 2.0;
  public static final double GEAR_RATIO = 25.0;

  public static final double EXTEND_VOLTAGE = 3.0;

  // was negative, now positive new climber moves same direction for cage catcher and climb
  public static final double CLIMB_VOLTAGE = 3.0;

  public static final double RETRACT_VOLTAGE_SLOW = -3.0;
  public static final double RESET_VOLTAGE = 20.0;

  public static final double DRUM_DIAMETER = 1.0;
  public static final double CAGE_CATCHER_EXTEND_POS_INCHES = 5.0; // FIXME: arbitrary for now
  public static final double MAX_HEIGHT_INCHES = 15.0; // FIXME: tune with new climber
  public static final double MIN_HEIGHT_INCHES = 2.25; // FIXME: tune with new climber

  public static final double KP = 0.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double KS = 1.0;
  public static final double KV = 0.0;
  // potentially will be removed later
  public static final double KA = 0.0;
  public static final double KVEXP = 0.0;
  public static final double KAEXP = 0.0;
  public static final double KG = 0.0;
}
