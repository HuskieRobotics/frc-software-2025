package frc.robot.subsystems.climber;

public class ClimberConstants {
  public static final int CLIMBER_MOTOR_CAN_ID = 16;

  public static final boolean CLIMBER_MOTOR_INVERTED = false; // untested, told to change
  public static final double CLIMBER_CONTINUOUS_CURRENT_LIMIT = 40.0;
  public static final double CLIMBER_STATOR_CURRENT_LIMIT = 50.0;
  public static final double CLIMBER_PEAK_CURRENT_LIMIT = 60.0;
  public static final double CLIMBER_PEAK_CURRENT_DURATION = 2.0;
  public static final double GEAR_RATIO = 25.0;

  public static final double EXTEND_VOLTAGE = -3.0;

  public static final double CLIMB_VOLTAGE = 12.0;

  public static final double RELEASE_CAGE_CATCHER_VOLTAGE = 3.0;
  public static final double RETRACT_VOLTAGE_SLOW = 3.0;

  public static final double DRUM_DIAMETER = 1.0;
  public static final double CAGE_CATCHER_EXTEND_POS_INCHES = 3.75;
  public static final double MAX_HEIGHT_INCHES = 21.75; // 16 + 3.75 // +2 at worlds
  public static final double HARDSTOP_POSITION_INCHES = 30.0;

  public static final int CLIMBER_LIMIT_SWITCH_DIO_1 = 8;
  public static final int CLIMBER_LIMIT_SWITCH_DIO_2 = 9;
  public static final int CAGE_CATCHER_LIMIT_SWITCH_DIO = 4;
}
