package frc.robot.configs;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.SwerveConstants;

/** Most of this is copied from Artemis; update with actual values */
public class New2025RobotConfig extends RobotConfig {
  private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
  private static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
  private static final int FRONT_LEFT_MODULE_STEER_ENCODER = 25;
  private static final Angle FRONT_LEFT_MODULE_STEER_OFFSET = Rotations.of(0.374); // update

  private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
  private static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
  private static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 24;
  private static final Angle FRONT_RIGHT_MODULE_STEER_OFFSET = Rotations.of(0.042); // update

  private static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6;
  private static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
  private static final int BACK_LEFT_MODULE_STEER_ENCODER = 22;
  private static final Angle BACK_LEFT_MODULE_STEER_OFFSET = Rotations.of(-0.031); // update

  private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 2;
  private static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4;
  private static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23;
  private static final Angle BACK_RIGHT_MODULE_STEER_OFFSET = Rotations.of(-0.501); // update

  private static final int GYRO_ID = 26;

  private static final Mass MASS = Kilograms.of(54.000); // update
  private static final MomentOfInertia MOI = KilogramSquareMeters.of(6.5); // update
  private static final Distance TRACKWIDTH = Meters.of(0.584); // update
  private static final Distance WHEELBASE = Meters.of(0.584); // update
  private static final Distance WHEEL_RADIUS = Meters.of(0.096 / 2.0); // update with sysid routine
  private static final double WHEEL_COEFFICIENT_OF_FRICTION =
      1.15; // unchanged, no reason it shouldn't be different from last year

  private static final Distance ROBOT_WIDTH_WITH_BUMPERS = Meters.of(0.9); // update
  private static final Distance ROBOT_LENGTH_WITH_BUMPERS = Meters.of(0.9); // update

  private static final double COUPLE_RATIO = 3.200; // possibly needs to be updated for mk4ns

  /* PID Values, Update all of these based on sysid routine when new robot arrives */
  private static final double ANGLE_KP = 105.0;
  private static final double ANGLE_KI = 0.0;
  private static final double ANGLE_KD = 0.045;

  private static final double DRIVE_KP = 9.5;
  private static final double DRIVE_KI = 0.0;
  private static final double DRIVE_KD = 0.0;

  private static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(6.0); // update
  private static final LinearVelocity MAX_COAST_VELOCITY = MetersPerSecond.of(0.04); // update
  private static final double SLOW_MODE_MULTIPLIER = 0.7;

  private static final String CAN_BUS_NAME = "canbus1";

  private static final String CAMERA_NAME_0 = "OV2311FR";
  private static final String CAMERA_NAME_1 = "OV2311BR";
  private static final String CAMERA_NAME_2 = "OV2311FL";
  private static final String CAMERA_NAME_3 = "OV2311BL";

  // UPDATE ALL OF THESE CAMERA LOCATIONS, THEY WILL BE VERY DIFFERENT

  // Front right camera
  private static final Transform3d ROBOT_TO_CAMERA_0 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(11.064),
              Units.inchesToMeters(-10.778),
              Units.inchesToMeters(8.189)),
          new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(0)));
  // pitch 45 degrees

  // Back right camera
  private static final Transform3d ROBOT_TO_CAMERA_1 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10.778),
              Units.inchesToMeters(-11.064),
              Units.inchesToMeters(8.189)),
          new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(-90)));

  // Front left camera
  private static final Transform3d ROBOT_TO_CAMERA_2 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(10.778),
              Units.inchesToMeters(11.064),
              Units.inchesToMeters(8.189)),
          new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(90)));

  // Back left camera
  private static final Transform3d ROBOT_TO_CAMERA_3 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-11.064),
              Units.inchesToMeters(10.778),
              Units.inchesToMeters(8.189)),
          new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180)));

  @Override
  public boolean getPhoenix6Licensed() {
    return true;
  }

  @Override
  public double getSwerveAngleKP() {
    return ANGLE_KP;
  }

  @Override
  public double getSwerveAngleKI() {
    return ANGLE_KI;
  }

  @Override
  public double getSwerveAngleKD() {
    return ANGLE_KD;
  }

  @Override
  public double getSwerveDriveKP() {
    return DRIVE_KP;
  }

  @Override
  public double getSwerveDriveKI() {
    return DRIVE_KI;
  }

  @Override
  public double getSwerveDriveKD() {
    return DRIVE_KD;
  }

  @Override
  public SwerveConstants getSwerveConstants() {
    return null; // MAKE SWERVE CONSTANTS FOR MK4N L2 PLUS
  }

  @Override
  public int[] getSwerveDriveMotorCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_DRIVE_MOTOR,
      FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      BACK_LEFT_MODULE_DRIVE_MOTOR,
      BACK_RIGHT_MODULE_DRIVE_MOTOR
    };
  }

  @Override
  public int[] getSwerveSteerMotorCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_STEER_MOTOR,
      FRONT_RIGHT_MODULE_STEER_MOTOR,
      BACK_LEFT_MODULE_STEER_MOTOR,
      BACK_RIGHT_MODULE_STEER_MOTOR
    };
  }

  @Override
  public int[] getSwerveSteerEncoderCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_STEER_ENCODER,
      FRONT_RIGHT_MODULE_STEER_ENCODER,
      BACK_LEFT_MODULE_STEER_ENCODER,
      BACK_RIGHT_MODULE_STEER_ENCODER
    };
  }

  @Override
  public Angle[] getSwerveSteerOffsets() {
    return new Angle[] {
      FRONT_LEFT_MODULE_STEER_OFFSET,
      FRONT_RIGHT_MODULE_STEER_OFFSET,
      BACK_LEFT_MODULE_STEER_OFFSET,
      BACK_RIGHT_MODULE_STEER_OFFSET
    };
  }

  @Override
  public int getGyroCANID() {
    return GYRO_ID;
  }

  @Override
  public Distance getTrackwidth() {
    return TRACKWIDTH;
  }

  @Override
  public Distance getWheelbase() {
    return WHEELBASE;
  }

  @Override
  public Mass getMass() {
    return MASS;
  }

  @Override
  public Distance getWheelRadius() {
    return WHEEL_RADIUS;
  }

  @Override
  public LinearVelocity getRobotMaxVelocity() {
    return MAX_VELOCITY;
  }

  @Override
  public int getPneumaticsHubCANID() {
    return 0;
  }
}
