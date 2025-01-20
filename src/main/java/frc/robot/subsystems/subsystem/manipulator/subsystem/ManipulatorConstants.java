package frc.robot.subsystems.subsystem.manipulator.subsystem;

public class ManipulatorConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ManipulatorConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Manipulator";
  public static final boolean TESTING = false; //this boolean might be useful while testing 

  // to confirm the can id's for the motors and sensors, go to the robot software feature sheet 2025
  public static final int FUNNEL_MOTOR_CAN_ID = 12; 
  public static final int INDEXER_MOTOR_CAN_ID = 14; 
  
  public static final int FUNNEL_IR_SENSOR_CAN_ID = 0; //not currently in the fearure sheet
  public static final int INDEXER_IR_SENSOR_CAN_ID = 0; //not currently in the fearure sheet

  //not sure what this is???
  public static final double GEAR_RATIO = 100.0;
  
  //tunable PID constants for both motors -- ALL ARE TBD
  public static final double FUNNEL_MOTOR_KP = 0;
  public static final double FUNNEL_MOTOR_KI = 0;
  public static final double FUNNEL_MOTOR_KD = 0;
  public static final double FUNNEL_MOTOR_KS = 0;
  public static final double INDEXER_MOTOR_KP = 0;
  public static final double INDEXER_MOTOR_KI = 0;
  public static final double INDEXER_MOTOR_KD = 0;
  public static final double INDEXER_MOTOR_KS = 0;

//testable properties -- all are TBD
public static final double FUNNEL_MOTOR_VOLTAGE = 0.0;
public static final double INDEXER_MOTOR_VOLTAGE = 0.0;

public static final double FUNNEL_MOTOR_VELOCITY = 0.0;
public static final double INDEXER_MOTOR_VELOCITY = 0.0;

public static final double INDEXER_MOTOR_VELOCITY_WHILE_SHOOTING_CORAL = 0.0; //tbd
//the regular velocity for the indexer is while the coral is getting indexed, this variable is to control the velocity while ejecting the coral out of the manipulator
//the funnel should only have one velocity when it is intaking coral, but if the coral is jammed and needs to be ejected out thru the funnel it should have a different velocity then
public static final double FUNNEL_MOTOR_VELOCITY_WHILE_EJECTING_CORAL = 0.0; //tbd

//unsure if these properties are needed
public static final double FUNNEL_ROLLER_VELOCITY_TOLERANCE = 0.0;
public static  final double INDEXER_ROLLER_VELOCITY_TOLERANCE = 0.0;

  public static final double POSITION_PID_P = 0.0;
  public static final double POSITION_PID_I = 0;
  public static final double POSITION_PID_D = 0;
  public static final double POSITION_PID_PEAK_OUTPUT = 1.0;
  public static final double POSITION_FEEDFORWARD = 0;

  //current limits -- all are set to default values
  public static final double FUNNEL_MOTOR_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final double FUNNEL_MOTOR_PEAK_CURRENT_LIMIT = 50;
  public static final double FUNNEL_MOTOR_PEAK_CURRENT_DURATION = 0.5;

  public static final double INDEXER_MOTOR_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final double INDEXER_MOTOR_PEAK_CURRENT_LIMIT = 50;
  public static final double INDEXER_MOTOR_PEAK_CURRENT_DURATION = 0.5;

  //invert motors
  public static final boolean FUNNEL_MOTOR_INVERTED = false;
  public static final boolean INDEXER_MOTOR_INVERTED = false;

  //not sure what these are for
  public static final int TIMEOUT_MS = 30;
  public static final int SLOT_INDEX = 0;

  //timer that keeps track of how long the robot is in the indexing coral in maninpulator state, to determine if the coral is stuck
  public static final double MANIPULATOR_IN_INDEXING_CORAL_STATE = 0.0;

  //SHOULD I ADD A VARIABLE HERE TO THAT SAYS SOMETHING LIKE STATE MACHINE STATE THAT CONNECTS TO THE ENUM OF THE STATE MACHINE STATES?
}
