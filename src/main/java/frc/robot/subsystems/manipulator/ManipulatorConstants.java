package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class ManipulatorConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ManipulatorConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Manipulator";

  // invert motors
  public static final boolean FUNNEL_MOTOR_INVERTED = true;
  public static final boolean INDEXER_MOTOR_INVERTED = false;
  public static final boolean PIVOT_MOTOR_INVERTED = false;

  // to confirm the can id's for the motors and sensors, go to the robot software feature sheet 2025
  public static final int FUNNEL_MOTOR_ID = 14;

  public static final int FUNNEL_IR_SENSOR_ID = 2;

  public static final int FUNNEL_IR_BACKUP_SENSOR_ID = 3;

  public static final int INDEXER_MOTOR_ID = 12;

  public static final int INDEXER_IR_SENSOR_ID = 0;

  public static final int INDEXER_IR_BACKUP_SENSOR_ID = 1;

  public static final int PIVOT_MOTOR_ID = 57; //TBD -- created for pivot motor

  public static final int ALGAE_IR_SENSOR_ID = 4; //new algae ir sensor --> id tbd
  
  public static final int ALGAE_IR_BACKUP_SENSOR_ID = 5;

  // not sure what this is???
  public static final double GEAR_RATIO_FUNNEL = 9.0;

  public static final double GEAR_RATIO_MANIPULATOR = 1.0;

  public static final double GEAR_RATIO_PIVOT = 0; //new for pivot motor

  public static final double MANIPULATOR_LENGTH = 5.0; //FIXME: update for sim

  public static final double MANIPULATOR_MASS = 0.0; //FIXME: update for sim

  // tunable PID constants for both motors -- ALL ARE TBD
  public static final double FUNNEL_MOTOR_KP = 0;
  public static final double FUNNEL_MOTOR_KI = 0;
  public static final double FUNNEL_MOTOR_KD = 0;
  public static final double FUNNEL_MOTOR_KS = 0;
  public static final double FUNNEL_MOTOR_KV = 0.05;
  public static final double FUNNEL_MOTOR_KA = 0.01;

  public static final double INDEXER_MOTOR_KP = 0;
  public static final double INDEXER_MOTOR_KI = 0;
  public static final double INDEXER_MOTOR_KD = 0;
  public static final double INDEXER_MOTOR_KS = 0;
  public static final double INDEXER_MOTOR_KV = 0.05;
  public static final double INDEXER_MOTOR_KA = 0.01;

  //pid values for pivot motor -- ALL TBD
  public static final double PIVOT_MOTOR_KP = 0;
  public static final double PIVOT_MOTOR_KI = 0;
  public static final double PIVOT_MOTOR_KD = 0;
  public static final double PIVOT_MOTOR_KS = 0;
  public static final double PIVOT_MOTOR_KV = 0;
  public static final double PIVOT_MOTOR_KA = 0;

  public static final double PIVOT_MOTOR_KV_EXPO = 0.0;

  public static final double PIVOT_MOTOR_KA_EXPO = 0.0;

  // FIXME: tune these values
  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_COLLECTING_CORAL = 4.0;
  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_SHOOTING_CORAL = 4.0;
  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_EJECTING_CORAL = -12.0;
  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_SHOOTING_CORAL_OUT_FUNNEL = -12;
  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_HOLDING_ALGAE = 0.0; //new constant for indexer motor when holding algae\
  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_SHOOTING_ALGAE_BARGE = -1.0; //FIXME:update value
  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_SHOOTING_ALGAE_PROCESSOR = -1.0; //FIXME:update value
  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_SHOOTING_ALGAE = 0.0; //FIXME:update value

  public static final double FUNNEL_MOTOR_VOLTAGE_WHILE_COLLECTING_CORAL = 4.0;
  public static final double FUNNEL_MOTOR_VOLTAGE_WHILE_EJECTING_CORAL = -10.0;
  public static final double FUNNEL_MOTOR_VOLTAGE_WHILE_SHOOTING_CORAL_OUT_FUNNEL = -12;

  public static final double INDEXER_MOTOR_VELOCITY_WHILE_SHOOTING_CORAL = 0.0; // tbd

  public static final double INDEXER_MOTOR_VELOCITY_WHILE_EJECTING_CORAL = 0.0; // tbd

  // the regular velocity for the indexer is while the coral is getting indexed, this variable is to
  // control the velocity while ejecting the coral out of the manipulator
  // the funnel should only have one velocity when it is intaking coral, but if the coral is jammed
  // and needs to be ejected out thru the funnel it should have a different velocity then

  public static final double FUNNEL_MOTOR_VELOCITY_WHILE_EJECTING_CORAL = 0.0; // tbd

   // this will be the set value to which the indexer motor velocity should be when removing
  // algae

   

  public static final double FUNNEL_MOTOR_VELOCITY_WHILE_COLLECTING_CORAL = 0.0;

  public static final double INDEXER_MOTOR_VELOCITY_WHILE_COLLECTING_CORAL = 0.0;

  //new setpoints for pivot motor for algae claw -- ALL TBD
  public static final Angle PIVOT_MOTOR_STARTING_POS = Degrees.of(90);
  public static final Angle PIVOT_MOTOR_AT_REEF_POS =  Degrees.of(0);
  public static final Angle PIVOT_MOTOR_SCORING_IN_PROCESSOR =  Degrees.of(0);
  public static final Angle PIVOT_MOTOR_SCORING_IN_BARGE =  Degrees.of(0);

  // used for timer
  public static final double CORAL_COLLECTION_TIME_OUT = 4.0;

  // timeout for letting funnel ramp up and whole funnel scoring timeout
  // *TEMPORARY: we might make a "reverse state machine" for scoring the funnel
  public static final double FUNNEL_RAMP_UP_TIMEOUT = 0.15;
  public static final double FUNNEL_SCORING_TIMEOUT = 2.0;

  public static final double EJECT_CORAL_DURATION_SECONDS = 2.5;

  public static final double INTAKE_ALGAE_TIMEOUT = 3.0; //FIXME:update value

  // current limits -- all are set to default values
  public static final double FUNNEL_MOTOR_PEAK_CURRENT_LIMIT = 40;

  public static final double INDEXER_MOTOR_PEAK_CURRENT_LIMIT = 40;

  public static final double PIVOT_MOTOR_PEAK_CURRENT_LIMIT = 40; //tbd for new pivot motor

  public static final double THRESHOLD_FOR_CURRENT_SPIKE = 35.0;

  public static final double THRESHOLD_CURRENT_SPIKE_ALGAE = 30.0; //FIXME:update value


  public static final Angle PIVOT_POSITION_TOLERANCE = Degrees.of(0.0); //FIXME:update value

}
