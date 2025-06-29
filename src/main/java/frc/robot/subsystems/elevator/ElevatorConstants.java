package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {

  public static final boolean IS_INVERTED = true;

  public static final double TOLERANCE_INCHES = 0.25;

  public static final Distance MAX_HEIGHT = Inches.of(74);

  public static final Distance MIN_HEIGHT = Inches.of(0.0);

  public static final Distance JUST_ABOVE_HARDSTOP = Inches.of(1.0); // set hardstop a bit above 0

  public static final double RESET_TOLERANCE = 0.25;

  public static final Distance HEIGHT_SWITCH_SLOT0 = Inches.of(20); // FIXME: Update these values
  public static final Distance HEIGHT_SWITCH_SLOT1 = Inches.of(40); // FIXME: Update these values

  public static final double PULLEY_CIRCUMFERENCE_INCHES = 5.9055;
  public static final int GEAR_RATIO = 5;

  public static final double ELEVATOR_MASS_KG = 4.5;

  public static final int LEAD_MOTOR_ID = 10;
  public static final int FOLLOWER_MOTOR_ID = 11;

  public static final boolean DEBUGGING = true;
  public static final boolean TESTING = true;

  public static final String SUBSYSTEM_NAME = "Elevator";

  public static final double ELEVATOR_LOWERING_VOLTAGE = -2.0;

  public static final double ELEVATOR_RAISE_SLOW_VOLTAGE = 2.0;

  public static final double ELEVATOR_LOWERING_SLOW_VOLTAGE = -2.0;

  public static final double KP_SLOT0 = 40.0;
  public static final double KI_SLOT0 = 0;
  public static final double KD_SLOT0 = 0;
  public static final double KS_SLOT0 = 0.01;
  public static final double KV_SLOT0 = 0.67505;
  public static final double KA_SLOT0 = 0.027564;
  public static final double KG_SLOT0 = 0.33833;

  public static final double KP_SLOT1 = 40.0;
  public static final double KI_SLOT1 = 0;
  public static final double KD_SLOT1 = 0;
  public static final double KS_SLOT1 = 0.01;
  public static final double KV_SLOT1 = 0.67505;
  public static final double KA_SLOT1 = 0.027564;
  public static final double KG_SLOT1 = 0.33833;

  public static final double KP_SLOT2 = 40.0;
  public static final double KI_SLOT2 = 0;
  public static final double KD_SLOT2 = 0;
  public static final double KS_SLOT2 = 0.01;
  public static final double KV_SLOT2 = 0.67505;
  public static final double KA_SLOT2 = 0.027564;
  public static final double KG_SLOT2 = 0.33833;

  public static final double KV_EXPO = 0.6;

  // was 0.05 with no funnel or climber on robot, caused wheels to leave ground
  // arbitrary increase for now
  public static final double KA_EXPO = 0.15; // 0.2

  public static final double CRUISE_VELOCITY = 0;

  // This is the current we watch for to detect that the elevator is completely down against the
  // hard stop
  public static final double STALL_CURRENT = 40.0;

  // This is the current we watch for to detect that the elevator is jammed and needs to be stopped
  public static final double JAMMED_CURRENT = 59.0;

  public enum ScoringHeight {
    HARDSTOP,

    L1,
    ABOVE_L1,
    L2,
    L3,
    L4,

    MAX_L2,
    MAX_L3,

    LOW_ALGAE,
    HIGH_ALGAE,

    BELOW_LOW_ALGAE,
    BELOW_HIGH_ALGAE,

    BARGE,
    PROCESSOR
  }

  /*
   * Highest point of each reef branch in inches
   */

  public static final Distance L1_HEIGHT = Inches.of(15.0);
  public static final Distance ABOVE_L1_HEIGHT = Inches.of(35.0);

  public static final Distance L2_HEIGHT = Inches.of(30);
  public static final Distance FAR_L2_HEIGHT = Inches.of(40);

  public static final Distance L3_HEIGHT = Inches.of(45);
  public static final Distance FAR_L3_HEIGHT = Inches.of(55); // 51 before
  public static final Distance L4_HEIGHT = Inches.of(71);

  public static final Distance BARGE_HEIGHT = Inches.of(74);
  public static final Distance PROCESSOR_HEIGHT = Inches.of(0);

  public static final Double FAR_SCORING_DISTANCE = Units.inchesToMeters(10.0); // 9
  public static final Double MIN_FAR_SCORING_DISTANCE = Units.inchesToMeters(3.0);

  public static final Double FAR_SCORING_Y_TOLERANCE = Units.inchesToMeters(6.0); // 5
  public static final Rotation2d FAR_SCORING_THETA_TOLERANCE = Rotation2d.fromDegrees(5.0);

  public static final Distance BELOW_HIGH_ALGAE_HEIGHT = Inches.of(20.0);
  public static final Distance HIGH_ALGAE_HEIGHT = Inches.of(30.0);

  public static final Distance BELOW_LOW_ALGAE_HEIGHT = Inches.of(5.0);
  public static final Distance LOW_ALGAE_HEIGHT = Inches.of(15.0);

  public static final double ELEVATOR_PEAK_CURRENT_LIMIT = 60.0;
}
