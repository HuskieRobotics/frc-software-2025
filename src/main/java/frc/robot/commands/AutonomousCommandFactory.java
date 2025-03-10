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
import frc.lib.team3061.drivetrain.DrivetrainConstants;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.vision.Vision;
import frc.lib.team6328.util.FieldConstants;
import frc.robot.Field2d;
import frc.robot.Field2d.Side;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ReefBranch;
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

    NamedCommands.registerCommand(
        "Raise Elevator",
        Commands.sequence(
            Commands.print("Raising Elevator"),
            Commands.waitUntil(manipulator::hasIndexedCoral),
            Commands.runOnce(() -> elevator.goToPosition(ElevatorConstants.ReefBranch.L4))));

    /************ Two Piece Left ************
     *
     * 2 corals scored J L4; L L4
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

    /** Three Piece Left 3 Coral Scored J, K, L L4 */
    Command threePieceLeft = getThreeCoralLeftCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("3 Piece Left", threePieceLeft);

    Command threePieceRight = getThreeCoralRightCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("3 Piece Right", threePieceRight);

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

  // FIXME: remove elevator setpoints in code since we have event markers now
  public Command getTwoCoralLeftAutoCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    PathPlannerPath collectCoralAfterJ2L;
    PathPlannerPath scoreCoralL2L;
    PathPlannerPath collectCoralAfterL2L;
    try {
      collectCoralAfterJ2L = PathPlannerPath.fromPathFile("#2 Collect Coral After J 2L");
      scoreCoralL2L = PathPlannerPath.fromPathFile("#3 Score Coral L 2L");
      collectCoralAfterL2L = PathPlannerPath.fromPathFile("#4 Collect Coral After L 2L");
    } catch (Exception e) {
      pathFileMissingAlert.setText(
          "Could not find the specified path file in getTwoCoralLeftAutoCommand.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }

    return Commands.sequence(
        Commands.parallel(
            Commands.sequence(
                Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2)), vision),
                new DriveToReef(
                    drivetrain,
                    () -> Field2d.getInstance().getNearestBranch(Side.RIGHT),
                    manipulator::setReadyToScore,
                    elevator::setDistanceFromReef,
                    new Transform2d(
                        DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
                        DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
                        Rotation2d.fromDegrees(
                            DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
                    3.0)),
            Commands.runOnce(
                /* possibly add a slight wait here before going up to l3 */
                () -> elevator.goToPosition(ElevatorConstants.ReefBranch.L3), elevator)),
        Commands.parallel(
            Commands.runOnce(
                () -> elevator.goToPosition(ElevatorConstants.ReefBranch.L4), elevator),
            Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ReefBranch.L4))),
        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3)), vision),
        Commands.runOnce(manipulator::shootCoral, manipulator),
        Commands.waitUntil(() -> !manipulator.hasCoral()),
        Commands.runOnce(
            () -> elevator.goToPosition(ElevatorConstants.ReefBranch.HARDSTOP), elevator),
        AutoBuilder.followPath(collectCoralAfterJ2L),
        getCollectCoralCommand(manipulator),
        Commands.parallel(
            AutoBuilder.followPath(scoreCoralL2L),
            Commands.waitUntil(manipulator::hasIndexedCoral)),
        getScoreL4Command(drivetrain, vision, manipulator, elevator, Side.RIGHT),
        AutoBuilder.followPath(collectCoralAfterL2L),
        getCollectCoralCommand(manipulator));
  }

  public Command getTwoCoralRightAutoCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    PathPlannerPath collectCoralAfterF2R;
    PathPlannerPath scoreCoralD2R;
    PathPlannerPath collectCoralAfterD2R;

    try {
      collectCoralAfterF2R = PathPlannerPath.fromPathFile("#2 Collect Coral After F 2R");
      scoreCoralD2R = PathPlannerPath.fromPathFile("#3 Score Coral D 2R");
      collectCoralAfterD2R = PathPlannerPath.fromPathFile("#4 Collect Coral After D 2R");
    } catch (Exception e) {
      pathFileMissingAlert.setText(
          "Could not find the specified path file in getTwoCoralRightAutoCommand.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }

    return Commands.sequence(
        Commands.parallel(
            Commands.sequence(
                Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2)), vision),
                new DriveToReef(
                    drivetrain,
                    () -> Field2d.getInstance().getNearestBranch(Side.RIGHT),
                    manipulator::setReadyToScore,
                    elevator::setDistanceFromReef,
                    new Transform2d(
                        DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
                        DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
                        Rotation2d.fromDegrees(
                            DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
                    3.0)),
            Commands.runOnce(
                /* possibly add a slight wait here before raising elevator */
                () -> elevator.goToPosition(ElevatorConstants.ReefBranch.L3), elevator)),
        Commands.parallel(
            Commands.runOnce(
                () -> elevator.goToPosition(ElevatorConstants.ReefBranch.L4), elevator),
            Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ReefBranch.L4))),
        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3)), vision),
        Commands.runOnce(manipulator::shootCoral, manipulator),
        Commands.waitUntil(() -> !manipulator.hasCoral()),
        Commands.runOnce(
            () -> elevator.goToPosition(ElevatorConstants.ReefBranch.HARDSTOP), elevator),
        AutoBuilder.followPath(collectCoralAfterF2R),
        getCollectCoralCommand(manipulator),
        Commands.parallel(
            AutoBuilder.followPath(scoreCoralD2R),
            Commands.waitUntil(manipulator::hasIndexedCoral)),
        getScoreL4Command(drivetrain, vision, manipulator, elevator, Side.RIGHT),
        AutoBuilder.followPath(collectCoralAfterD2R),
        getCollectCoralCommand(manipulator));
  }

  public Command getThreeCoralLeftCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    PathPlannerPath scoreCoralK;
    PathPlannerPath collectCoralAfterK;

    try {
      scoreCoralK = PathPlannerPath.fromPathFile("#5 Score Coral K 3L");
      collectCoralAfterK = PathPlannerPath.fromPathFile("#6 Collect Coral After K 3L");
    } catch (Exception e) {
      pathFileMissingAlert.setText(
          "Could not find the specified path file in getThreeCoralLeftAutoCommand.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }

    return Commands.sequence(
        getTwoCoralLeftAutoCommand(drivetrain, vision, manipulator, elevator),
        Commands.parallel(
            AutoBuilder.followPath(scoreCoralK), Commands.waitUntil(manipulator::hasIndexedCoral)),
        getScoreL4Command(drivetrain, vision, manipulator, elevator, Side.LEFT),
        AutoBuilder.followPath(collectCoralAfterK));
  }

  public Command getThreeCoralRightCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    PathPlannerPath scoreCoralC;
    PathPlannerPath collectCoralAfterC;

    try {
      scoreCoralC = PathPlannerPath.fromPathFile("#5 Score Coral C 3R");
      collectCoralAfterC = PathPlannerPath.fromPathFile("#6 Collect Coral After C 3R");
    } catch (Exception e) {
      pathFileMissingAlert.setText(
          "Could not find the specified path file in getThreeCoralRightAutoCommand.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }

    return Commands.sequence(
        getTwoCoralRightAutoCommand(drivetrain, vision, manipulator, elevator),
        Commands.parallel(
            AutoBuilder.followPath(scoreCoralC), Commands.waitUntil(manipulator::hasIndexedCoral)),
        getScoreL4Command(drivetrain, vision, manipulator, elevator, Side.LEFT),
        AutoBuilder.followPath(collectCoralAfterC));
  }

  public Command getOneCoralCenterCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    PathPlannerPath backUpH1C;
    try {
      backUpH1C = PathPlannerPath.fromPathFile("Back Up H 1C");
    } catch (Exception e) {
      pathFileMissingAlert.setText(
          "Could not find the specified path file in getOneCoralCenterCommand.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }

    return Commands.sequence(
        Commands.runOnce(() -> elevator.goToPosition(ElevatorConstants.ReefBranch.L4), elevator),
        getScoreL4Command(drivetrain, vision, manipulator, elevator, Side.RIGHT),
        getDescoreAlgaeCommand(drivetrain, vision, manipulator, elevator),
        AutoBuilder.followPath(backUpH1C));
  }

  private Command getScoreL4Command(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator, Side side) {
    return Commands.sequence(
        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2)), vision),
        Commands.parallel(
            Commands.runOnce(
                () -> elevator.goToPosition(ElevatorConstants.ReefBranch.L4), elevator),
            new DriveToReef(
                drivetrain,
                () -> Field2d.getInstance().getNearestBranch(side),
                manipulator::setReadyToScore,
                elevator::setDistanceFromReef,
                new Transform2d(
                    DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
                    DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
                    Rotation2d.fromDegrees(DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
                1.6),
            Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ReefBranch.L4))),
        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3)), vision),
        Commands.waitSeconds(0.2), // ADD WAIT TO SEE IF NOT WAITING WAS IMPEDING PRECISION
        Commands.runOnce(manipulator::shootCoral, manipulator),
        Commands.waitUntil(() -> !manipulator.hasCoral()),
        Commands.runOnce(
            () -> elevator.goToPosition(ElevatorConstants.ReefBranch.HARDSTOP), elevator));
  }

  public Command getDescoreAlgaeCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    return Commands.parallel(
        Commands.runOnce(manipulator::removeAlgae),
        Commands.sequence(
            Commands.runOnce(() -> elevator.goToPosition(ReefBranch.BELOW_ALGAE_1)),
            Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2))),
            Commands.waitUntil(() -> elevator.isAtPosition(ReefBranch.BELOW_ALGAE_1)),
            new DriveToReef(
                drivetrain,
                () -> Field2d.getInstance().getNearestBranch(Side.REMOVE_ALGAE),
                manipulator::setReadyToScore,
                elevator::setDistanceFromReef,
                new Transform2d(
                    DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
                    DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
                    Rotation2d.fromDegrees(DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
                0.5),
            Commands.runOnce(() -> elevator.goToPosition(ReefBranch.ABOVE_ALGAE_1)),
            Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3))),
            Commands.waitUntil(() -> elevator.isAtPosition(ReefBranch.ABOVE_ALGAE_1)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(manipulator::algaeIsRemoved)));
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
