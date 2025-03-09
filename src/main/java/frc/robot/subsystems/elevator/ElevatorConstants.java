package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

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

  public static final double PULLY_CIRCUMFERANCE_INCHES = 5.9055;
  public static final int GEAR_RATIO = 5;

  public static final double ELEVATOR_MASS_KG = 4.5; // FIXEME: Update this value

  public static final int LEAD_MOTOR_ID = 10;
  public static final int FOLLOWER_MOTOR_ID = 11;

  public static final boolean DEBUGGING = true;
  public static final boolean TESTING = true;

  public static final String SUBSYSTEM_NAME = "Elevator";

  public static final double ELEVATOR_LOWERING_VOLTAGE = -2.0; // FIXME: Update this value

  public static final double ELEVATOR_RAISE_SLOW_VOLTAGE = 2.0; // FIXME: Update this value

  public static final double ELEVATOR_LOWERING_SLOW_VOLTAGE = -2.0; // FIXME: Update this value

  // FIXME: Update all K values

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
  public static final double KA_EXPO = 0.2;

  public static final double CRUISE_VELOCITY = 0;

  public static final double STALL_CURRENT = 40.0;

  public enum ReefBranch {
    HARDSTOP,

    L1,
    ABOVE_L1,
    L2,
    L3,
    L4,

    MAX_L2,
    MAX_L3,

    ALGAE_1,
    ALGAE_2,

    BELOW_ALGAE_1,
    ABOVE_ALGAE_1,

    BELOW_ALGAE_2,
    ABOVE_ALGAE_2
  }

  /*
   * Highest point of each reef branch in inches
   */

  public static final Distance L1_HEIGHT = Inches.of(15.0);
  public static final Distance ABOVE_L1_HEIGHT = Inches.of(25.0);

  public static final Distance L2_HEIGHT = Inches.of(30); // 1 coral away 35
  public static final Distance FAR_L2_HEIGHT = Inches.of(37); // tune

  public static final Distance L3_HEIGHT = Inches.of(45); // 1 coral away 51
  public static final Distance FAR_L3_HEIGHT = Inches.of(52); // tune
  public static final Distance L4_HEIGHT = Inches.of(71);

  public static final Double FAR_SCORING_DISTANCE = Units.inchesToMeters(6.0);

  public static final Distance ALGAE1_HEIGHT =
      Inches.of(13.0); // height under is 9 // height of impact is 13
  public static final Distance ALGAE2_HEIGHT =
      Inches.of(28.0); // height under is 24 // height of impact is 28

  public static final Distance ABOVE_ALGAE_2_HEIGHT = Inches.of(34.0);
  public static final Distance BELOW_ALGAE_2_HEIGHT = Inches.of(20.0);

  public static final Distance BELOW_ALGAE_1_HEIGHT = Inches.of(7.0);
  public static final Distance ABOVE_ALGAE_1_HEIGHT = Inches.of(16.0);

  public static final double ELEVATOR_PEAK_CURRENT_LIMIT = 60.0;
}
