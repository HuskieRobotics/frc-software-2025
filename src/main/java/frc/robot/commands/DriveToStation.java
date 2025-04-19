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
import frc.lib.team3061.drivetrain.DrivetrainConstants;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Field2d;
import frc.robot.subsystems.manipulator.Manipulator;
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
public class DriveToStation extends Command {
  private final Drivetrain drivetrain;
  private final Manipulator manipulator;
  private Supplier<Pose2d> poseSupplier;
  private Pose2d targetPose;
  private Transform2d targetTolerance;

  private boolean firstRun = true;

  private double timeout;

  private Timer timer;

  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber(
          "DriveToStation/DriveKp", RobotConfig.getInstance().getDriveToPoseDriveKP());

  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber(
          "DriveToStation/DriveKd", RobotConfig.getInstance().getDriveToPoseDriveKD());
  private static final LoggedTunableNumber driveKi =
      new LoggedTunableNumber("DriveToStation/DriveKi", 0);
  private static final LoggedTunableNumber thetaKp =
      new LoggedTunableNumber(
          "DriveToStation/ThetaKp", RobotConfig.getInstance().getDriveToPoseThetaKP());
  private static final LoggedTunableNumber thetaKd =
      new LoggedTunableNumber(
          "DriveToStation/ThetaKd", RobotConfig.getInstance().getDriveToPoseThetaKD());
  private static final LoggedTunableNumber thetaKi =
      new LoggedTunableNumber(
          "DriveToStation/ThetaKi", RobotConfig.getInstance().getDriveToPoseThetaKI());

  private static final LoggedTunableNumber closeVelocityBoost =
      new LoggedTunableNumber("DriveToStation/close velocity boost", 0.5);

  private final PIDController xController =
      new PIDController(driveKp.get(), driveKi.get(), driveKd.get(), LOOP_PERIOD_SECS);
  private final PIDController yController =
      new PIDController(driveKp.get(), driveKi.get(), driveKd.get(), LOOP_PERIOD_SECS);
  private final PIDController thetaController =
      new PIDController(thetaKp.get(), thetaKi.get(), thetaKd.get(), LOOP_PERIOD_SECS);

  /**
   * Constructs a new DriveToStation command that drives the robot in a straight line to the
   * specified pose. A pose supplier is specified instead of a pose since the target pose may not be
   * known when this command is created.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param poseSupplier a supplier that returns the pose to drive to
   */
  public DriveToStation(
      Drivetrain drivetrain,
      Manipulator manipulator,
      Supplier<Pose2d> poseSupplier,
      Transform2d tolerance,
      double timeout) {
    this.drivetrain = drivetrain;
    this.manipulator = manipulator;
    this.poseSupplier = poseSupplier;
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

    firstRun = true;

    drivetrain.enableAccelerationLimiting();

    Logger.recordOutput("DriveToStation/isFinished", false);
    Logger.recordOutput("DriveToStation/withinTolerance", false);

    this.timer.restart();
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the
   * velocities based on the current and target poses and invokes the drivetrain subsystem's drive
   * method.
   */
  @Override
  public void execute() {

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

    Logger.recordOutput("DriveToStation/x velocity (field frame)", xVelocity);
    Logger.recordOutput("DriveToStation/y velocity (field frame)", yVelocity);

    // convert the pose difference and velocities into the reef frame
    Transform2d robotRelativeDifference = new Transform2d(targetPose, drivetrain.getPose());
    var robotRelativeVelocities =
        new Translation2d(xVelocity, yVelocity).rotateBy(targetPose.getRotation().unaryMinus());

    // add 0.5 to the robot relative x velocity to make sure we run into it
    robotRelativeVelocities =
        new Translation2d(
            robotRelativeVelocities.getX() + DrivetrainConstants.DRIVE_TO_STATION_X_BOOST,
            robotRelativeVelocities.getY());

    Logger.recordOutput("DriveToStation/targetPose", targetPose);

    // y boost if our bumpers are aligned to the station already and need to get to the target spot
    if (Math.abs(robotRelativeDifference.getX()) < 0.0762) {
      Logger.recordOutput("DriveToStation/boost velocity", true);
      if (robotRelativeDifference.getY() > 0) {
        robotRelativeVelocities =
            new Translation2d(
                robotRelativeVelocities.getX(),
                robotRelativeVelocities.getY() - closeVelocityBoost.get());
      } else if (robotRelativeDifference.getY() < 0) {
        robotRelativeVelocities =
            new Translation2d(
                robotRelativeVelocities.getX(),
                robotRelativeVelocities.getY() + closeVelocityBoost.get());
      }
    } else {
      Logger.recordOutput("DriveToStation/boost velocity", false);
    }

    Logger.recordOutput(
        "DriveToStation/x velocity (robot relative)", robotRelativeVelocities.getX());
    Logger.recordOutput(
        "DriveToStation/y velocity (robot relative)", robotRelativeVelocities.getY());

    // convert the velocities back into the field frame
    var fieldRelativeVelocities = robotRelativeVelocities.rotateBy(targetPose.getRotation());

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
        "DriveToStation/difference",
        new Transform2d(
            drivetrain.getPose().getX() - targetPose.getX(),
            drivetrain.getPose().getY() - targetPose.getY(),
            Rotation2d.fromRadians(
                drivetrain.getPose().getRotation().getRadians()
                    - targetPose.getRotation().getRadians())));

    // convert the pose difference and velocities into the reef frame
    Transform2d robotRelativeDifference = new Transform2d(targetPose, drivetrain.getPose());
    Logger.recordOutput("DriveToStation/difference (robot relative)", robotRelativeDifference);

    boolean atGoal =
        Math.abs(robotRelativeDifference.getX()) < targetTolerance.getX()
            && Math.abs(robotRelativeDifference.getY()) < targetTolerance.getY()
            && Math.abs(robotRelativeDifference.getRotation().getRadians())
                < targetTolerance.getRotation().getRadians();

    if (atGoal) {
      Logger.recordOutput("DriveToStation/withinTolerance", true);
    }

    boolean cannotReachTargetPose = false;
    Logger.recordOutput("DriveToStation/cannotReachTargetPose", cannotReachTargetPose);
    // if (firstRun) {
    //   firstRun = false;
    //   cannotReachTargetPose = robotRelativeDifference.getX() > 0.05;
    //   if (cannotReachTargetPose) {
    //     drivetrain.setDriveToPoseCanceled(true);
    //   }
    // }

    // check that each of the controllers is at their goal or if the timeout is elapsed
    // check if it is physically possible for us to drive to the selected position without going
    // through the reef (sign of our x difference)
    return manipulator.indexingCoral()
        || !drivetrain.isMoveToPoseEnabled()
        || this.timer.hasElapsed(timeout)
        || atGoal;
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
    Logger.recordOutput("DriveToStation/isFinished", true);
  }
}
