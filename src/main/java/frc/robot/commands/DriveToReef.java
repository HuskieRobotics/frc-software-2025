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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Field2d;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive to the specified pose in
 * a straight line. The execute method invokes the drivetrain subsystem's drive method. For
 * following a predetermined path, refer to the FollowPath Command class. For generating a path on
 * the fly and following that path, refer to the MoveToPose Command class.
 *
 * <p>This is an adapted version of the generic DriveToPose command that is specifically adapted for
 * the reef. This includes a velocity boost when the robot is close to the reef.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: the robot is at the specified pose (within the specified tolerances)
 *
 * <p>At End: stops the drivetrain
 */
public class DriveToReef extends Command {
  private final Drivetrain drivetrain;
  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<Boolean> onTarget;
  private Pose2d targetPose;
  private Transform2d targetTolerance;

  private double timeout;

  private Timer timer;

  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber(
          "DriveToReef/DriveKp", RobotConfig.getInstance().getDriveToPoseDriveKP());
  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber(
          "DriveToReef/DriveKd", RobotConfig.getInstance().getDriveToPoseDriveKD());
  private static final LoggedTunableNumber driveKi =
      new LoggedTunableNumber("DriveToReef/DriveKi", 0);
  private static final LoggedTunableNumber thetaKp =
      new LoggedTunableNumber(
          "DriveToReef/ThetaKp", RobotConfig.getInstance().getDriveToPoseThetaKP());
  private static final LoggedTunableNumber thetaKd =
      new LoggedTunableNumber(
          "DriveToReef/ThetaKd", RobotConfig.getInstance().getDriveToPoseThetaKD());
  private static final LoggedTunableNumber thetaKi =
      new LoggedTunableNumber(
          "DriveToReef/ThetaKi", RobotConfig.getInstance().getDriveToPoseThetaKI());

  private static final LoggedTunableNumber closeVelocityBoost =
      new LoggedTunableNumber("DriveToReef/close velocity boost", 0.5);

  private final PIDController xController =
      new PIDController(driveKp.get(), driveKi.get(), driveKd.get(), LOOP_PERIOD_SECS);
  private final PIDController yController =
      new PIDController(driveKp.get(), driveKi.get(), driveKd.get(), LOOP_PERIOD_SECS);
  private final PIDController thetaController =
      new PIDController(thetaKp.get(), thetaKi.get(), thetaKd.get(), LOOP_PERIOD_SECS);

  /**
   * Constructs a new DriveToReef command that drives the robot in a straight line to the specified
   * pose. A pose supplier is specified instead of a pose since the target pose may not be known
   * when this command is created.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param poseSupplier a supplier that returns the pose to drive to
   */
  public DriveToReef(
      Drivetrain drivetrain,
      Supplier<Pose2d> poseSupplier,
      Consumer<Boolean> onTargetConsumer,
      Transform2d tolerance,
      double timeout) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    this.onTarget = onTargetConsumer;
    this.targetTolerance = tolerance;
    this.timer = new Timer();
    this.timeout = timeout;
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

    Logger.recordOutput("DriveToReef/targetPose", targetPose);
    Logger.recordOutput("DriveToReef/isFinished", false);
    Logger.recordOutput("DriveToReef/withinTolerance", false);

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
          yController.setPID(pid[0], pid[1], pid[2]);
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
    double yVelocity = yController.calculate(currentPose.getY(), this.targetPose.getY());
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), this.targetPose.getRotation().getRadians());

    Logger.recordOutput("DriveToReef/x velocity (field frame)", xVelocity);
    Logger.recordOutput("DriveToReef/y velocity (field frame)", yVelocity);

    // convert the pose difference and velocities into the reef frame
    Transform2d reefRelativeDifference = new Transform2d(targetPose, drivetrain.getPose());
    var reefRelativeVelocities =
        new Translation2d(xVelocity, yVelocity).rotateBy(targetPose.getRotation().unaryMinus());

    // add 0.25 to the reef relative x velocity to make sure we run into it
    reefRelativeVelocities =
        new Translation2d(reefRelativeVelocities.getX() + 0.25, reefRelativeVelocities.getY());

    if (Math.abs(reefRelativeDifference.getX()) < 0.0762) {
      Logger.recordOutput("DriveToReef/boost velocity", true);
      if (reefRelativeDifference.getY() > 0) {
        reefRelativeVelocities =
            new Translation2d(
                reefRelativeVelocities.getX(),
                reefRelativeVelocities.getY() - closeVelocityBoost.get());
      } else if (reefRelativeDifference.getY() < 0) {
        reefRelativeVelocities =
            new Translation2d(
                reefRelativeVelocities.getX(),
                reefRelativeVelocities.getY() + closeVelocityBoost.get());
      }
    } else {
      Logger.recordOutput("DriveToReef/boost velocity", false);
    }

    Logger.recordOutput("DriveToReef/x velocity (reef frame)", reefRelativeVelocities.getX());
    Logger.recordOutput("DriveToReef/y velocity (reef frame)", reefRelativeVelocities.getY());

    // convert the velocities back into the field frame
    var fieldRelativeVelocities = reefRelativeVelocities.rotateBy(targetPose.getRotation());

    int allianceMultiplier = Field2d.getInstance().getAlliance() == Alliance.Blue ? 1 : -1;

    drivetrain.drive(
        allianceMultiplier * fieldRelativeVelocities.getX(),
        allianceMultiplier * fieldRelativeVelocities.getY(),
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
    Logger.recordOutput(
        "DriveToReef/difference",
        new Transform2d(
            drivetrain.getPose().getX() - targetPose.getX(),
            drivetrain.getPose().getY() - targetPose.getY(),
            Rotation2d.fromRadians(
                drivetrain.getPose().getRotation().getRadians()
                    - targetPose.getRotation().getRadians())));

    // convert the pose difference and velocities into the reef frame
    Transform2d reefRelativeDifference = new Transform2d(targetPose, drivetrain.getPose());
    Logger.recordOutput("DriveToReef/difference (reef frame)", reefRelativeDifference);

    boolean atGoal =
        Math.abs(reefRelativeDifference.getX()) < targetTolerance.getX()
            && Math.abs(reefRelativeDifference.getY()) < targetTolerance.getY()
            && Math.abs(reefRelativeDifference.getRotation().getRadians())
                < targetTolerance.getRotation().getRadians();

    if (atGoal) {
      onTarget.accept(true);
      Logger.recordOutput("DriveToReef/withinTolerance", true);
    } else if (!drivetrain.isMoveToPoseEnabled() || this.timer.hasElapsed(timeout)) {
      onTarget.accept(false);
    }

    // check that each of the controllers is at their goal or if the timeout is elapsed
    return !drivetrain.isMoveToPoseEnabled() || this.timer.hasElapsed(timeout) || atGoal;
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
    Logger.recordOutput("DriveToReef/isFinished", true);
  }
}
