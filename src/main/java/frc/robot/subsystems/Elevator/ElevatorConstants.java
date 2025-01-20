package frc.robot.subsystems.Elevator;


import edu.wpi.first.units.Units.*;

public class ElevatorConstants {



  public static final double TOLERANCE = 0;
  public static final double MAX_HEIGHT = 0.0; // FIXME
  public static final double CONVERSION_FACTOR = 0.0;// FIXME
  public static final int GEAR_RATIO = 5; //CHECK THIS VALUE
  public static final double OFFSET = 0.0; // FIXME


  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = false;
  public static final String SUBSYSTEM_NAME = "Elevator";


  // FIXME: Update all K values

  public static final double KP_SLOT0 = 0;
  public static final double KI_SLOT0 = 0;
  public static final double KD_SLOT0 = 0;
  public static final double KS_SLOT0 = 0;
  public static final double KV_SLOT0 = 0;
  public static final double KA_SLOT0 = 0;
  public static final double KV_EXPO_SLOT0 = 0;
  public static final double KA_EXPO_SLOT0 = 0;
  public static final double KG_SLOT0 = 0;

  public static final double KP_SLOT1 = 0;
  public static final double KI_SLOT1 = 0;
  public static final double KD_SLOT1 = 0;
  public static final double KS_SLOT1 = 0;
  public static final double KV_SLOT1 = 0;
  public static final double KA_SLOT1 = 0;
  public static final double KV_EXPO_SLOT1 = 0;
  public static final double KA_EXPO_SLOT1 = 0;
  public static final double KG_SLOT1 = 0;

  public static final double KP_SLOT2 = 0;
  public static final double KI_SLOT2 = 0;
  public static final double KD_SLOT2 = 0;
  public static final double KS_SLOT2 = 0;
  public static final double KV_SLOT2 = 0;
  public static final double KA_SLOT2 = 0;
  public static final double KV_EXPO_SLOT2 = 0;
  public static final double KA_EXPO_SLOT2 = 0;
  public static final double KG_SLOT2 = 0;

  public static final double CRUISE_VELOCITY = 0;
  public static final double ACCELERATION = 0;
  public static final double JERK = 0;




  public enum ReefBranch{

    HARDSTOP,

    L1,
    L2,
    L3,
    L4,

    ALGAE_1,
    ALGAE_2,
  }

  /*
   * Highest point of each reef branch in inches
   */

   Distance armLength = Inches.of(32.25);

  public static final Distance L1_HEIGHT = new Distance(15.94, edu.wpi.first.wpilibj.units.length.inch); 
  public static final Distance L2_HEIGHT = 31.89;
  public static final Distance L3_HEIGHT = 47.64;
  public static final Distance L4_HEIGHT = 72.05;

  public static final Distance ALGAE1_HEIGHT = 0.0; // FIXME: Update these values
  public static final Distance ALGAE2_HEIGHT = 0.0; // FIXME: Update these values

}


