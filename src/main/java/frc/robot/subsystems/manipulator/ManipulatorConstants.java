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

  public static final int FUNNEL_IR_SENSOR_ID = 3;

  public static final int FUNNEL_IR_BACKUP_SENSOR_ID = 2;

  public static final int INDEXER_MOTOR_ID = 56;

  public static final int INDEXER_IR_SENSOR_ID = 0;

  public static final int INDEXER_IR_BACKUP_SENSOR_ID = 1;

  public static final int PIVOT_MOTOR_ID = 57;

  public static final int ALGAE_IR_SENSOR_ID = 6;

  public static final int ALGAE_IR_BACKUP_SENSOR_ID = 7;

  // not sure what this is???
  public static final double GEAR_RATIO_FUNNEL = 9.0;

  public static final double GEAR_RATIO_MANIPULATOR = 1.0;

  public static final double GEAR_RATIO_PIVOT = 3.0;

  public static final double MANIPULATOR_LENGTH_METERS = 0.264;

  public static final double MANIPULATOR_MASS_KG = 0.9789;

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

  // pid values for pivot motor -- ALL TBD
  public static final double PIVOT_MOTOR_KP = 10.0;
  public static final double PIVOT_MOTOR_KI = 0;
  public static final double PIVOT_MOTOR_KD = 0;
  public static final double PIVOT_MOTOR_KS = 0;
  public static final double PIVOT_MOTOR_KV = 0;
  public static final double PIVOT_MOTOR_KA = 0;
  public static final double PIVOT_MOTOR_KG = 0.0;

  public static final double PIVOT_MOTOR_KV_EXPO = 0.36;

  public static final double PIVOT_MOTOR_KA_EXPO = 2.0;

  public static final double PIVOT_EXTEND_CURRENT = -15.0;
  public static final double PIVOT_RETRACT_UP_CURRENT = 20;
  public static final double PIVOT_RETRACT_HOLD_CURRENT = 10;

  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_COLLECTING_CORAL = 4.0;

  // Fast voltage: L1, L4, far L2, far L3
  public static final double INDEXER_VOLTAGE_SHOOT_FAST = 4.0;

  // Slow voltage: close L2, close L3
  public static final double INDEXER_VOLTAGE_SHOOT_SLOW = 3.0;

  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_EJECTING_CORAL = -12.0;
  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_SHOOTING_CORAL_OUT_FUNNEL = -12;
  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_HOLDING_ALGAE = 2.0;
  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_SHOOTING_ALGAE_BARGE = -10.0;
  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_SHOOTING_ALGAE_PROCESSOR = -10.0;
  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_DROPPING_ALGAE =
      -6.0; // FIXME:update value

  public static final double INDEXER_MOTOR_VOLTAGE_WHILE_COLLECTING_ALGAE = 4.0;

  public static final double FUNNEL_MOTOR_VOLTAGE_WHILE_COLLECTING_CORAL = 4.0;
  public static final double FUNNEL_MOTOR_VOLTAGE_WHILE_EJECTING_CORAL = -10.0;
  public static final double FUNNEL_MOTOR_VOLTAGE_WHILE_SHOOTING_CORAL_OUT_FUNNEL = -12;

  public static final double INDEXER_MOTOR_VELOCITY_WHILE_SHOOTING_CORAL = 0.0; // tbd

  public static final double INDEXER_MOTOR_VELOCITY_WHILE_EJECTING_CORAL = 0.0; // tbd

  // use velocity control on funnel later
  public static final double FUNNEL_MOTOR_VELOCITY_WHILE_COLLECTING_CORAL = 0.0;
  public static final double FUNNEL_MOTOR_VELOCITY_WHILE_EJECTING_CORAL = 0.0;

  // new setpoints for pivot motor for algae claw -- ALL TBD
  public static final Angle PIVOT_MOTOR_STARTING_POS = Degrees.of(90);
  public static final Angle PIVOT_MOTOR_AT_REEF_POS = Degrees.of(0);

  // used for timer
  public static final double CORAL_COLLECTION_TIME_OUT = 4.0;

  public static final double EJECT_CORAL_DURATION_SECONDS = 2.5;

  public static final double INTAKE_ALGAE_TIMEOUT = 3.0; // FIXME: update value
  public static final double BARGE_ALGAE_TIMEOUT = 1.0; // FIXME: tune to be lower if we need
  public static final double PROCESSOR_ALGAE_TIMEOUT = 1.0; // FIXME: tune to be lower if we need

  // current limits -- all are set to default values
  public static final double FUNNEL_MOTOR_PEAK_CURRENT_LIMIT = 40;

  public static final double INDEXER_MOTOR_PEAK_CURRENT_LIMIT = 40;

  public static final double PIVOT_MOTOR_PEAK_CURRENT_LIMIT = 40;

  public static final double THRESHOLD_FOR_CURRENT_SPIKE = 35.0;

  public static final double THRESHOLD_CURRENT_SPIKE_ALGAE = 30.0; // FIXME: tune
}
