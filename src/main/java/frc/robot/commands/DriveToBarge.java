// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// originally from https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Field2d;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive to the specified pose in
 * a straight line. The execute method invokes the drivetrain subsystem's drive method. For
 * following a predetermined path, refer to the FollowPath Command class. For generating a path on
 * the fly and following that path, refer to the MoveToPose Command class.
 *
 * <p>This is a more generic command that drives to any given pose, and can be customized for other
 * specific goals.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: the robot is at the specified pose (within the specified tolerances)
 *
 * <p>At End: stops the drivetrain
 */
public class DriveToBarge extends Command {
  private final Drivetrain drivetrain;
  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<Boolean> onTarget;
  private Pose2d targetPose;

  private Timer timer;

  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber(
          "DriveToPose/DriveKp", RobotConfig.getInstance().getDriveToPoseDriveKP());
  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber(
          "DriveToPose/DriveKd", RobotConfig.getInstance().getDriveToPoseDriveKD());
  private static final LoggedTunableNumber driveKi =
      new LoggedTunableNumber("DriveToPose/DriveKi", 0);
  private static final LoggedTunableNumber thetaKp =
      new LoggedTunableNumber(
          "DriveToPose/ThetaKp", RobotConfig.getInstance().getDriveToPoseThetaKP());
  private static final LoggedTunableNumber thetaKd =
      new LoggedTunableNumber(
          "DriveToPose/ThetaKd", RobotConfig.getInstance().getDriveToPoseThetaKD());
  private static final LoggedTunableNumber thetaKi =
      new LoggedTunableNumber(
          "DriveToPose/ThetaKi", RobotConfig.getInstance().getDriveToPoseThetaKI());

  private final PIDController xController =
      new PIDController(driveKp.get(), driveKi.get(), driveKd.get(), LOOP_PERIOD_SECS);
  private final PIDController thetaController =
      new PIDController(thetaKp.get(), thetaKi.get(), thetaKd.get(), LOOP_PERIOD_SECS);

  private DoubleSupplier translationYSupplier;
  private BooleanSupplier interrupted;

  /**
   * Constructs a new DriveToBarge command that drives the robot in a straight line to the specified
   * pose. A pose supplier is specified instead of a pose since the target pose may not be known
   * when this command is created.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param poseSupplier a supplier that returns the pose to drive to
   */

  /*
   * DriveToBarge should be customized to not lock out sideways translational motion, as it doesn't matter where on the barge we want to score.
   */
  public DriveToBarge(
      Drivetrain drivetrain,
      Supplier<Pose2d> poseSupplier,
      Consumer<Boolean> onTargetConsumer,
      BooleanSupplier interrupted,
      DoubleSupplier translationYSupplier) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    this.interrupted = interrupted;
    this.onTarget = onTargetConsumer;
    this.timer = new Timer();
    this.translationYSupplier = translationYSupplier;
    addRequirements(drivetrain);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * This method is invoked once when this command is scheduled. It resets all the PID controllers
   * and initializes the current and target poses. It is critical that this initialization occurs in
   * this method and not the constructor as this object is constructed well before the command is
   * scheduled and the robot's pose will definitely have changed and the target pose may not be
   * known until this command is scheduled.
   */
  @Override
  public void initialize() {
    // Reset all controllers
    this.targetPose = poseSupplier.get();

    drivetrain.enableAccelerationLimiting();

    Logger.recordOutput("DriveToBarge/targetPose", targetPose);
    Logger.recordOutput("DriveToBarge/isFinished", false);
    Logger.recordOutput("DriveToBarge/withinTolerance", false);

    this.timer.restart();
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the
   * velocities based on the current and target poses and invokes the drivetrain subsystem's drive
   * method.
   */
  @Override
  public void execute() {

    LEDs.getInstance().requestState(LEDs.States.AUTO_DRIVING_TO_SCORE);

    // Update from tunable numbers
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          xController.setPID(pid[0], pid[1], pid[2]);
        },
        driveKp,
        driveKi,
        driveKd);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> thetaController.setPID(pid[0], pid[1], pid[2]),
        thetaKp,
        thetaKi,
        thetaKd);

    Pose2d currentPose = drivetrain.getPose();

    // use last values of filter
    double xVelocity = xController.calculate(currentPose.getX(), this.targetPose.getX());
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), this.targetPose.getRotation().getRadians());

    Logger.recordOutput("DriveToBarge/x velocity (field relative)", xVelocity);

    int allianceMultiplier = Field2d.getInstance().getAlliance() == Alliance.Blue ? 1 : -1;

    drivetrain.drive(
        allianceMultiplier * xVelocity,
        translationYSupplier.getAsDouble(),
        thetaVelocity,
        true,
        true);
  }

  /**
   * This method returns true if the command has finished. It is invoked periodically while this
   * command is scheduled (after execute is invoked). This command is considered finished if the
   * move-to-pose feature is disabled on the drivetrain subsystem or if the timeout has elapsed or
   * if all the PID controllers are at their goal.
   *
   * @return true if the command has finished
   */
  @Override
  public boolean isFinished() {
    // We will probably not need this target, but just logging for now just in case.
    Transform2d difference =
        new Transform2d(
            drivetrain.getPose().getX() - targetPose.getX(),
            drivetrain.getPose().getY() - targetPose.getY(),
            Rotation2d.fromRadians(
                drivetrain.getPose().getRotation().getRadians()
                    - targetPose.getRotation().getRadians()));

    Logger.recordOutput("DriveToBarge/difference", difference);

    Transform2d robotRelativeDifference = new Transform2d(targetPose, drivetrain.getPose());
    Logger.recordOutput("DriveToBarge/difference (robot relative)", robotRelativeDifference);

    boolean atGoal = interrupted.getAsBoolean();

    // check each of the controllers is at their goal
    return !drivetrain.isMoveToPoseEnabled() || atGoal;
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.disableAccelerationLimiting();
    drivetrain.stop();
    Logger.recordOutput("DriveToBarge/isFinished", true);
  }
}
