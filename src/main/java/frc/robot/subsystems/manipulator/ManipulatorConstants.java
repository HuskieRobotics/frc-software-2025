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

  public static final double FUNNEL_GEAR_RATIO = 9.0;
  public static final double MANIPULATOR_GEAR_RATIO = 1.0;
  public static final double PIVOT_GEAR_RATIO = 3.0;

  public static final double MANIPULATOR_LENGTH_METERS = 0.264;
  public static final double MANIPULATOR_MASS_KG = 0.9789;

  // tunable PID constants for both motors -- ALL ARE TBD
  public static final double FUNNEL_KP = 0;
  public static final double FUNNEL_KI = 0;
  public static final double FUNNEL_KD = 0;
  public static final double FUNNEL_KS = 0;
  public static final double FUNNEL_KV = 0.05;
  public static final double FUNNEL_KA = 0.01;

  public static final double INDEXER_KP = 8.0;
  public static final double INDEXER_KI = 0;
  public static final double INDEXER_KD = 0;
  public static final double INDEXER_KS = 0.5;
  public static final double INDEXER_KV = 0.05;
  public static final double INDEXER_KA = 0.01;

  public static final double PIVOT_KP = 10.0;
  public static final double PIVOT_KI = 0;
  public static final double PIVOT_KD = 0;
  public static final double PIVOT_KS = 0;
  public static final double PIVOT_KV = 0;
  public static final double PIVOT_KA = 0;
  public static final double PIVOT_KG = 0.0;

  public static final double PIVOT_EXTEND_CURRENT = -15.0;
  public static final double PIVOT_RETRACT_UP_CURRENT = 20;
  public static final double PIVOT_RETRACT_HOLD_CURRENT = 10;

  public static final double INDEXER_COLLECTION_VOLTAGE = 4.0;

  public static final double INDEXER_HOLD_ALGAE_CURRENT = 30.0;
  public static final double INDEXER_SHOOT_ALGAE_BARGE_CURRENT = -35.0;
  public static final double INDEXER_SHOOT_ALGAE_PROCESSOR_CURRENT = -35.0;
  public static final double INDEXER_DROP_ALGAE_CURRENT = -35.0;

  // Fast voltage: L1, L4, far L2, far L3
  public static final double INDEXER_SHOOT_FAST_VOLTAGE = 4.0;
  // Slow voltage: close L2, close L3
  public static final double INDEXER_SHOOT_SLOW_VOLTAGE = 3.0;

  public static final double INDEXER_EJECT_VOLTAGE = -12.0;
  public static final double INDEXER_COLLECT_ALGAE_VOLTAGE = 4.0;

  // deprecated constants -- using current control
  // FIXME: remove once current control is confirmed
  public static final double INDEXER_HOLD_ALGAE_VOLTAGE = 2.0;
  public static final double INDEXER_SHOOT_ALGAE_BARGE_VOLTAGE = -10.0;
  public static final double INDEXER_SHOOT_ALGAE_PROCESSOR_VOLTAGE = -10.0;

  public static final double FUNNEL_COLLECTION_VOLTAGE = 4.0;
  public static final double FUNNEL_EJECT_VOLTAGE = -10.0;

  // in place for if velocity control gets used
  public static final double FUNNEL_COLLECTION_VELOCITY = 0.0;
  public static final double FUNNEL_EJECT_VELOCITY = 0.0;

  // new setpoints for pivot motor for algae claw -- ALL TBD
  public static final Angle PIVOT_MOTOR_STARTING_POS = Degrees.of(90);
  public static final Angle PIVOT_MOTOR_AT_REEF_POS = Degrees.of(0);

  // used for timer
  public static final double CORAL_COLLECTION_TIME_OUT = 4.0;
  public static final double EJECT_CORAL_DURATION_SECONDS = 2.5;

  public static final double INTAKE_ALGAE_TIMEOUT = 3.0;
  public static final double BARGE_ALGAE_TIMEOUT = 0.25;
  public static final double PROCESSOR_ALGAE_TIMEOUT = 0.3; // FIXME: tune to be lower if we need
  public static final double DROP_ALGAE_TIMEOUT = 0.2;

  // current limits and spike thresholds
  public static final double FUNNEL_MOTOR_PEAK_CURRENT_LIMIT = 40;
  public static final double INDEXER_MOTOR_PEAK_CURRENT_LIMIT = 40;
  public static final double PIVOT_MOTOR_PEAK_CURRENT_LIMIT = 40;
  public static final double CORAL_CURRENT_SPIKE_THRESHOLD = 35.0;
  public static final double ALGAE_CURRENT_SPIKE_THRESHOLD = 30.0;

  public static final double PIVOT_CURRENT_TOLERANCE_AMPS = 1.0;
}
