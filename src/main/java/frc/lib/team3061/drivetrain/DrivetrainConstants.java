package frc.lib.team3061.drivetrain;

import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private DrivetrainConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Drivetrain";

  public static final boolean ENABLE_TELEPORT_DETECTION = false;
  public static final double TELEPORT_DETECTION_THRESHOLD_METERS = 0.4;

  public static final double DRIVE_TO_REEF_X_TOLERANCE = Units.inchesToMeters(0.5);
  public static final double DRIVE_TO_REEF_Y_TOLERANCE = Units.inchesToMeters(0.5);
  public static final double DRIVE_TO_REEF_THETA_TOLERANCE_DEG = 2.0;

  public static final double DEMO_MODE_MAX_VELOCITY = 0.5;

  public static final double TILT_THRESHOLD_DEG = 5.0;
  public static final double UNTILT_VELOCITY_MPS = 0.5;

  public enum SysIDCharacterizationMode {
    TRANSLATION_VOLTS,
    TRANSLATION_CURRENT,
    STEER_VOLTS,
    STEER_CURRENT,
    ROTATION_VOLTS
  }
}
