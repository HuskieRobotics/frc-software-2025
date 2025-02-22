package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.vision.Vision;
import frc.lib.team6328.util.FieldConstants;
import frc.robot.Field2d;
import frc.robot.Field2d.Side;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.manipulator.Manipulator;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousCommandFactory {

  private static AutonomousCommandFactory autonomousCommandFactory = null;

  // arbitrary, find the actual starting poses
  private Pose2d blueLeftStartingAutoPose =
      new Pose2d(7.017, 6.076, Rotation2d.fromDegrees(-132.957));
  private Pose2d blueRightStartingAutoPose =
      new Pose2d(6.972, 1.884, Rotation2d.fromDegrees(145.333));
  private Pose2d redLeftStartingAutoPose = FlippingUtil.flipFieldPose(blueLeftStartingAutoPose);
  private Pose2d redRightStartingAutoPose = FlippingUtil.flipFieldPose(blueRightStartingAutoPose);

  // set arbitrary tolerance values to 3 inches in each direction and 5 degrees
  private Transform2d autoStartTolerance =
      new Transform2d(
          Units.inchesToMeters(3),
          Units.inchesToMeters(3),
          new Rotation2d(Units.degreesToRadians(5)));

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  private final Alert pathFileMissingAlert =
      new Alert("Could not find the specified path file.", AlertType.kError);

  /**
   * Returns the singleton instance of this class.
   *
   * @return the singleton instance of this class
   */
  public static AutonomousCommandFactory getInstance() {
    if (autonomousCommandFactory == null) {
      autonomousCommandFactory = new AutonomousCommandFactory();
    }
    return autonomousCommandFactory;
  }

  private AutonomousCommandFactory() {}

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void configureAutoCommands(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    /************* PathPlanner Named Commands *****************/
    NamedCommands.registerCommand(
        "Score Left L4", getScoreL4Command(drivetrain, vision, manipulator, elevator, Side.LEFT));
    NamedCommands.registerCommand(
        "Score Right L4", getScoreL4Command(drivetrain, vision, manipulator, elevator, Side.RIGHT));
    NamedCommands.registerCommand("Collect Coral", getCollectCoralCommand(manipulator));

    /************ Two Piece Left ************
     *
     * 2 corals scored J L4; K L4
     *
     */

    Command twoPieceLeft = getTwoCoralLeftAutoCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("2 Piece Left", twoPieceLeft);

    /************ Two Piece Right ************
     *
     * 2 corals scored E L4; D L4
     *
     */
    Command twoPieceRight = getTwoCoralRightAutoCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("2 Piece Right", twoPieceRight);

    /************ One Piece Center ************
     *
     * 1 coral scored H L4
     *
     */
    Command onePieceCenter = getOneCoralCenterCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("1 Piece Center", onePieceCenter);

    /************ Start Point ************
     *
     * useful for initializing the pose of the robot to a known location
     *
     */

    Command startPoint =
        Commands.runOnce(
            () -> {
              try {
                drivetrain.resetPose(
                    PathPlannerPath.fromPathFile("Start Point").getStartingDifferentialPose());
              } catch (Exception e) {
                pathFileMissingAlert.setText("Could not find the specified path file: Start Point");
                pathFileMissingAlert.set(true);
              }
            },
            drivetrain);
    autoChooser.addOption("Start Point", startPoint);

    /************ Distance Test ************
     *
     * used for empirically determining the wheel radius
     *
     */
    autoChooser.addOption(
        "Distance Test Slow", createTuningAutoPath("DistanceTestSlow", true, drivetrain));
    autoChooser.addOption(
        "Distance Test Med", createTuningAutoPath("DistanceTestMed", true, drivetrain));
    autoChooser.addOption(
        "Distance Test Fast", createTuningAutoPath("DistanceTestFast", true, drivetrain));

    /************ Auto Tuning ************
     *
     * useful for tuning the autonomous PID controllers
     *
     */
    autoChooser.addOption(
        "Rotation Test Slow", createTuningAutoPath("RotationTestSlow", false, drivetrain));
    autoChooser.addOption(
        "Rotation Test Fast", createTuningAutoPath("RotationTestFast", false, drivetrain));

    autoChooser.addOption(
        "Oval Test Slow", createTuningAutoPath("OvalTestSlow", false, drivetrain));
    autoChooser.addOption(
        "Oval Test Fast", createTuningAutoPath("OvalTestFast", false, drivetrain));

    /************ Drive Velocity Tuning ************
     *
     * useful for tuning the drive velocity PID controller
     *
     */
    autoChooser.addOption("Drive Velocity Tuning", this.getDriveVelocityTuningCommand(drivetrain));

    /************ Swerve Rotation Tuning ************
     *
     * useful for tuning the swerve module rotation PID controller
     *
     */
    autoChooser.addOption(
        "Swerve Rotation Tuning", this.getSwerveRotationTuningCommand(drivetrain));

    /************ Drive Wheel Radius Characterization ************
     *
     * useful for characterizing the drive wheel Radius
     *
     */
    autoChooser.addOption( // start by driving slowing in a circle to align wheels
        "Drive Wheel Radius Characterization",
        this.getDriveWheelRadiusCharacterizationCommand(drivetrain));
  }

  private Command getDriveVelocityTuningCommand(Drivetrain drivetrain) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
        Commands.repeatingSequence(
            Commands.deadline(
                Commands.waitSeconds(1.0),
                Commands.run(() -> drivetrain.drive(2.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                Commands.run(() -> drivetrain.drive(-0.5, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(3.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(() -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(-3.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(() -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain))));
  }

  private Command getSwerveRotationTuningCommand(Drivetrain drivetrain) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
        Commands.repeatingSequence(
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(0.1, 0.1, 0.0, true, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(-0.1, 0.1, 0.0, true, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(-0.1, -0.1, 0.0, true, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(0.1, -0.1, 0.0, true, false), drivetrain))));
  }

  private Command getDriveWheelRadiusCharacterizationCommand(Drivetrain drivetrain) {
    return CharacterizationCommands.wheelRadiusCharacterization(drivetrain);
  }

  public Command getTwoCoralLeftAutoCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    try {
      PathPlannerPath scoreCoralJ2BL = PathPlannerPath.fromPathFile("#1 Score Coral J 2BL");
      PathPlannerPath collectCoralJ2BL = PathPlannerPath.fromPathFile("#2 Collect Coral J 2BL");
      PathPlannerPath scoreCoralL2BL = PathPlannerPath.fromPathFile("#3 Score Coral L 2BL");
      PathPlannerPath collectCoralL2BL = PathPlannerPath.fromPathFile("#4 Collect Coral L 2BL");

      return Commands.sequence(
          Commands.parallel(
              AutoBuilder.followPath(scoreCoralJ2BL),
              Commands.runOnce(
                  () -> elevator.goToPosition(ElevatorConstants.ReefBranch.L4), elevator)),
          getScoreL4Command(drivetrain, vision, manipulator, elevator, Side.RIGHT),
          AutoBuilder.followPath(collectCoralJ2BL),
          getCollectCoralCommand(manipulator),
          Commands.parallel(
              AutoBuilder.followPath(scoreCoralL2BL),
              Commands.sequence(
                  Commands.waitUntil(manipulator::hasIndexedCoral),
                  Commands.runOnce(
                      () -> elevator.goToPosition(ElevatorConstants.ReefBranch.L4), elevator))),
          getScoreL4Command(drivetrain, vision, manipulator, elevator, Side.RIGHT),
          AutoBuilder.followPath(collectCoralL2BL),
          getCollectCoralCommand(manipulator));

    } catch (Exception e) {
      pathFileMissingAlert.setText(
          "Could not find the specified path file in getTwoCoralLeftAutoCommand.");
      pathFileMissingAlert.set(true);

      return Commands.waitSeconds(0);
    }
  }

  public Command getTwoCoralRightAutoCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    try {
      PathPlannerPath scoreCoralE2BR = PathPlannerPath.fromPathFile("#1 Score Coral F 2BR");
      PathPlannerPath collectCoralE2BR = PathPlannerPath.fromPathFile("#2 Collect Coral F 2BR");
      PathPlannerPath scoreCoralD2BR = PathPlannerPath.fromPathFile("#3 Score Coral D 2BR");
      PathPlannerPath collectCoralD2BR = PathPlannerPath.fromPathFile("#4 Collect Coral D 2BR");

      return Commands.sequence(
          Commands.parallel(
              AutoBuilder.followPath(scoreCoralE2BR),
              Commands.runOnce(
                  () -> elevator.goToPosition(ElevatorConstants.ReefBranch.L4), elevator)),
          getScoreL4Command(drivetrain, vision, manipulator, elevator, Side.RIGHT),
          AutoBuilder.followPath(collectCoralE2BR),
          getCollectCoralCommand(manipulator),
          Commands.parallel(
              AutoBuilder.followPath(scoreCoralD2BR),
              Commands.sequence(
                  Commands.waitUntil(manipulator::hasIndexedCoral),
                  Commands.runOnce(
                      () -> elevator.goToPosition(ElevatorConstants.ReefBranch.L4), elevator))),
          getScoreL4Command(drivetrain, vision, manipulator, elevator, Side.RIGHT),
          AutoBuilder.followPath(collectCoralD2BR),
          getCollectCoralCommand(manipulator));

    } catch (Exception e) {
      pathFileMissingAlert.setText(
          "Could not find the specified path file in getTwoCoralRightAutoCommand.");
      pathFileMissingAlert.set(true);

      return Commands.waitSeconds(0);
    }
  }

  public Command getOneCoralCenterCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    try {
      PathPlannerPath backUpH1C = PathPlannerPath.fromPathFile("Back Up H 1C");
      return Commands.sequence(
          Commands.runOnce(() -> elevator.goToPosition(ElevatorConstants.ReefBranch.L4), elevator),
          getScoreL4Command(drivetrain, vision, manipulator, elevator, Side.RIGHT),
          AutoBuilder.followPath(backUpH1C));
    } catch (Exception e) {
      pathFileMissingAlert.setText(
          "Could not find the specified path file in getOneCoralCenterCommand.");
      pathFileMissingAlert.set(true);

      return Commands.waitSeconds(0);
    }
  }

  private Command getScoreL4Command(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator, Side side) {
    return Commands.sequence(
        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2)), vision),
        Commands.parallel(
            new DriveToPose(
                drivetrain,
                () -> Field2d.getInstance().getNearestBranch(side),
                manipulator::setReadyToScore,
                new Transform2d(
                    Units.inchesToMeters(2.0),
                    Units.inchesToMeters(1.0),
                    Rotation2d.fromDegrees(2.0))),
            Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ReefBranch.L4))),
        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3)), vision),
        Commands.runOnce(manipulator::shootCoral, manipulator),
        Commands.waitUntil(() -> !manipulator.hasCoral()),
        Commands.runOnce(
            () -> elevator.goToPosition(ElevatorConstants.ReefBranch.HARDSTOP), elevator));
  }

  // when programmed, this will wait until a coral is fully detected within the robot (use
  // manipulator state machine)
  private Command getCollectCoralCommand(Manipulator manipulator) {
    return Commands.waitUntil(manipulator::indexingCoral);
  }

  private Command createTuningAutoPath(
      String autoName, boolean measureDistance, Drivetrain drivetrain) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::captureInitialConditions),
        new PathPlannerAuto(autoName),
        Commands.runOnce(() -> drivetrain.captureFinalConditions(autoName, measureDistance)));
  }

  public boolean alignedToStartingPose() {

    // find the target position
    Transform2d difference;

    if (Field2d.getInstance().getAlliance() == Alliance.Blue) {
      difference =
          RobotOdometry.getInstance().getEstimatedPose().getY() > (FieldConstants.fieldWidth / 2.0)
              ? RobotOdometry.getInstance().getEstimatedPose().minus(blueLeftStartingAutoPose)
              : RobotOdometry.getInstance().getEstimatedPose().minus(blueRightStartingAutoPose);
    } else {
      difference =
          RobotOdometry.getInstance().getEstimatedPose().getY() > (FieldConstants.fieldWidth / 2.0)
              ? RobotOdometry.getInstance().getEstimatedPose().minus(redRightStartingAutoPose)
              : RobotOdometry.getInstance().getEstimatedPose().minus(redLeftStartingAutoPose);
    }

    boolean isAligned =
        Math.abs(difference.getX()) < autoStartTolerance.getX()
            && Math.abs(difference.getY()) < autoStartTolerance.getY()
            && Math.abs(difference.getRotation().getRadians())
                < autoStartTolerance.getRotation().getRadians();

    // this method will be invoked in something like our disabledPeriodic method
    Logger.recordOutput("PathFinding/alignedForAuto", isAligned);
    Logger.recordOutput("PathFinding/xDiff", difference.getX());
    Logger.recordOutput("PathFinding/yDiff", difference.getY());
    Logger.recordOutput("PathFinding/rotDiff", difference.getRotation().getDegrees());

    return isAligned;
  }
}
