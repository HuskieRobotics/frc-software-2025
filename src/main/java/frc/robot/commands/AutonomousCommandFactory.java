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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.drivetrain.DrivetrainConstants;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.vision.Vision;
import frc.lib.team6328.util.FieldConstants;
import frc.robot.Field2d;
import frc.robot.Field2d.Side;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.manipulator.Manipulator;
import java.util.List;
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

  private Timer timer;

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

  private AutonomousCommandFactory() {
    timer = new Timer();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void configureAutoCommands(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    // add commands to the auto chooser
    // autoChooser.addDefaultOption("Do Nothing", new InstantCommand().withName("Do Nothing"));

    NamedCommands.registerCommand(
        "Raise Elevator",
        Commands.sequence(
            Commands.print("Raising Elevator"),
            Commands.waitUntil(manipulator::hasIndexedCoral),
            Commands.runOnce(() -> elevator.goToPosition(ElevatorConstants.ScoringHeight.L4))));

    /************ One Piece Center ************
     *
     * 1 coral scored H L4
     *
     */
    Command onePieceCenter = getOneCoralCenterCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("1 Piece Center", onePieceCenter);

    /** Three Piece Left 3 Coral Scored J, K, L L4 */
    Command fourPieceLeft = getFourCoralLeftCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("4 Piece Left", fourPieceLeft);

    Command fourPieceRight = getFourCoralRightCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("4 Piece Right", fourPieceRight);

    Command autoAutoSelector =
        Commands.either(
            fourPieceRight,
            fourPieceLeft,
            () -> {
              if ((Field2d.getInstance().getAlliance() == Alliance.Blue
                      && RobotOdometry.getInstance().getEstimatedPose().getY()
                          < FieldConstants.fieldWidth / 2.0)
                  || (Field2d.getInstance().getAlliance() == Alliance.Red
                      && RobotOdometry.getInstance().getEstimatedPose().getY()
                          > FieldConstants.fieldWidth / 2.0)) {
                return true;
              } else {
                return false;
              }
            });
    autoChooser.addDefaultOption("Auto Auto Selector", autoAutoSelector);

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

  public Command getFourCoralLeftCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    PathPlannerPath collectCoralAfterJ2L;
    PathPlannerPath scoreCoralL2L;
    PathPlannerPath collectCoralAfterL2L;
    PathPlannerPath scoreCoralK;
    PathPlannerPath collectCoralAfterK;

    try {
      collectCoralAfterJ2L = PathPlannerPath.fromPathFile("#2 Collect Coral After J 2L");
      scoreCoralL2L = PathPlannerPath.fromPathFile("#3 Score Coral L 2L");
      collectCoralAfterL2L = PathPlannerPath.fromPathFile("#4 Collect Coral After L 2L");
      scoreCoralK = PathPlannerPath.fromPathFile("#5 Score Coral K 3L");
      collectCoralAfterK = PathPlannerPath.fromPathFile("#6 Collect Coral After K 3L");
    } catch (Exception e) {
      pathFileMissingAlert.setText(
          "Could not find the specified path file in getFourCoralLeftAutoCommand.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }

    return Commands.sequence(
        Commands.runOnce(() -> timer.restart()),
        getScoreFirstAutoCoralCommand(drivetrain, manipulator, elevator, vision, Side.RIGHT),
        getCollectAndScoreCommand(
            drivetrain,
            manipulator,
            elevator,
            vision,
            Side.RIGHT,
            collectCoralAfterJ2L,
            scoreCoralL2L),
        getCollectAndScoreCommand(
            drivetrain,
            manipulator,
            elevator,
            vision,
            Side.LEFT,
            collectCoralAfterL2L,
            scoreCoralK),
        Commands.either(
            getCollectAndScoreFourthCommand(
                drivetrain, manipulator, elevator, vision, Side.LEFT, collectCoralAfterK),
            Commands.none(), // if we can't collect the fourth coral, do nothing
            () -> (!timer.hasElapsed(12.0)) // only run if we have time left in auto
            ));
  }

  public Command getFourCoralRightCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    PathPlannerPath collectCoralAfterE2R;
    PathPlannerPath scoreCoralD2R;
    PathPlannerPath collectCoralAfterD2R;
    PathPlannerPath scoreCoralC;
    PathPlannerPath collectCoralAfterC;

    try {
      collectCoralAfterE2R = PathPlannerPath.fromPathFile("#2 Collect Coral After E 2R");
      scoreCoralD2R = PathPlannerPath.fromPathFile("#3 Score Coral D 2R");
      collectCoralAfterD2R = PathPlannerPath.fromPathFile("#4 Collect Coral After D 2R");
      scoreCoralC = PathPlannerPath.fromPathFile("#5 Score Coral C 3R");
      collectCoralAfterC = PathPlannerPath.fromPathFile("#6 Collect Coral After C 3R");
    } catch (Exception e) {
      pathFileMissingAlert.setText(
          "Could not find the specified path file in getFourCoralRightAutoCommand.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }

    return Commands.sequence(
        Commands.runOnce(() -> timer.restart()),
        getScoreFirstAutoCoralCommand(drivetrain, manipulator, elevator, vision, Side.LEFT),
        getCollectAndScoreCommand(
            drivetrain,
            manipulator,
            elevator,
            vision,
            Side.RIGHT,
            collectCoralAfterE2R,
            scoreCoralD2R),
        getCollectAndScoreCommand(
            drivetrain,
            manipulator,
            elevator,
            vision,
            Side.LEFT,
            collectCoralAfterD2R,
            scoreCoralC),
        Commands.either(
            getCollectAndScoreFourthCommand(
                drivetrain, manipulator, elevator, vision, Side.RIGHT, collectCoralAfterC),
            Commands.none(),
            () -> (!timer.hasElapsed(12.0))));
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
        Commands.runOnce(() -> elevator.goToPosition(ElevatorConstants.ScoringHeight.L4), elevator),
        getScoreL4Command(drivetrain, vision, manipulator, elevator, Side.RIGHT),
        CrossSubsystemsCommandsFactory.getCollectAlgaeCommand(
            drivetrain, manipulator, elevator, vision),
        AutoBuilder.followPath(backUpH1C));
  }

  private Command getScoreL4Command(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator, Side side) {
    return Commands.sequence(
        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2))),
        Commands.parallel(
            Commands.runOnce(
                () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.L4), elevator),
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
            Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ScoringHeight.L4))),
        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3))),
        Commands.waitSeconds(0.2), // ADD WAIT TO SEE IF NOT WAITING WAS IMPEDING PRECISION
        Commands.runOnce(manipulator::shootCoralFast, manipulator),
        Commands.waitUntil(() -> !manipulator.coralIsInManipulator()),
        Commands.runOnce(
            () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.HARDSTOP), elevator));
  }

  private Command getScoreFirstAutoCoralCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision, Side side) {
    return Commands.sequence(
        Commands.parallel(
            Commands.sequence(
                Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2)), vision),
                new DriveToReef(
                    drivetrain,
                    () -> Field2d.getInstance().getNearestBranch(side),
                    manipulator::setReadyToScore,
                    elevator::setDistanceFromReef,
                    new Transform2d(
                        DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
                        DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
                        Rotation2d.fromDegrees(
                            DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
                    3.0)),
            Commands.sequence(
                Commands.waitSeconds(0.5),
                Commands.runOnce(
                    () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.L4), elevator))),
        Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ScoringHeight.L4)),
        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3)), vision),
        Commands.waitSeconds(0.5),
        Commands.runOnce(manipulator::shootCoralFast, manipulator),
        Commands.waitUntil(() -> !manipulator.coralIsInManipulator()),
        Commands.runOnce(
            () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.HARDSTOP), elevator));
  }

  private Command getCollectAndScoreCommand(
      Drivetrain drivetrain,
      Manipulator manipulator,
      Elevator elevator,
      Vision vision,
      Side side,
      PathPlannerPath collect,
      PathPlannerPath score) {
    return Commands.sequence(
        AutoBuilder.followPath(collect),
        getCollectCoralCommand(manipulator),
        Commands.parallel(
            Commands.sequence(
                Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2)), vision),
                new DriveToReef(
                    drivetrain,
                    () -> Field2d.getInstance().getNearestBranch(side),
                    manipulator::setReadyToScore,
                    elevator::setDistanceFromReef,
                    new Transform2d(
                        DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
                        DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
                        Rotation2d.fromDegrees(
                            DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
                    4.0)),
            Commands.sequence(
                Commands.waitSeconds(0.5),
                Commands.waitUntil(manipulator::hasIndexedCoral),
                Commands.runOnce(
                    () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.L4), elevator))),
        Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ScoringHeight.L4)),
        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3)), vision),
        Commands.waitSeconds(0.5),
        Commands.runOnce(manipulator::shootCoralFast, manipulator),
        Commands.waitUntil(() -> !manipulator.coralIsInManipulator()),
        Commands.runOnce(
            () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.HARDSTOP), elevator));
  }

  private Command getCollectAndScoreFourthCommand(
      Drivetrain drivetrain,
      Manipulator manipulator,
      Elevator elevator,
      Vision vision,
      Side side,
      PathPlannerPath collect) {
    return Commands.sequence(AutoBuilder.followPath(collect), getCollectCoralCommand(manipulator));
    // ,Commands.parallel(
    //     Commands.sequence(
    //         Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2)), vision),
    //         new DriveToReef(
    //             drivetrain,
    //             () -> Field2d.getInstance().getFourthAutoCoralPose(side),
    //             manipulator::setReadyToScore,
    //             elevator::setDistanceFromReef,
    //             new Transform2d(
    //                 DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
    //                 DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
    //                 Rotation2d.fromDegrees(
    //                     DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
    //             4.0)),
    //     Commands.sequence(
    //         Commands.waitSeconds(0.5),
    //         Commands.waitUntil(manipulator::hasIndexedCoral),
    //         Commands.runOnce(
    //             () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.L2), elevator))),
    // Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ScoringHeight.L2)),
    // Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3)), vision),
    // Commands.runOnce(manipulator::shootCoralFast, manipulator),
    // Commands.waitUntil(() -> !manipulator.coralIsInManipulator()),
    // Commands.runOnce(
    //     () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.HARDSTOP), elevator));
  }

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
}
