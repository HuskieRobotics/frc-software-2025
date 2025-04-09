package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.drivetrain.DrivetrainConstants;
import frc.lib.team3061.vision.Vision;
import frc.robot.Field2d;
import frc.robot.Field2d.Side;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ScoringHeight;
import frc.robot.subsystems.manipulator.Manipulator;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousCommandFactory {

  private static AutonomousCommandFactory autonomousCommandFactory = null;

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
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand().withName("Do Nothing"));

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

    Command fourPieceCloseLeft =
        getFourCoralLeftCloseCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("4 Piece Left Close JKLA", fourPieceCloseLeft);

    Command fourPieceCloseRight =
        getFourCoralRightCloseCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("4 Piece Right Close EDCA", fourPieceCloseRight);

    Command fourPieceFarLeft =
        getFourCoralLeftFarCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("4 Piece Left Far IKLJ", fourPieceFarLeft);

    Command fourPieceFarRight =
        getFourCoralRightFarCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("4 Piece Right Far FDCE", fourPieceFarRight);

    Command oneCoralTwoAlgae =
        getOneCoralTwoAlgaeCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("1 Coral 2 Algae", oneCoralTwoAlgae);

    Command twoCoralBackLeft =
        getTwoCoralBackLeftCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("2 Coral Back Left", twoCoralBackLeft);

    Command twoCoralBackRight =
        getTwoCoralBackRightCommand(drivetrain, vision, manipulator, elevator);
    autoChooser.addOption("2 Coral Back Right", twoCoralBackRight);

    Command bumpAndThreeCoralLeft =
        getBumpAndThreeCoralLeftCommand(drivetrain, manipulator, elevator, vision);
    autoChooser.addOption("Bump and 3 Coral Left", bumpAndThreeCoralLeft);

    Command bumpAndThreeCoralRight =
        getBumpAndThreeCoralRightCommand(drivetrain, manipulator, elevator, vision);
    autoChooser.addOption("Bump and 3 Coral Right", bumpAndThreeCoralRight);

    // Command autoAutoSelector =
    //     Commands.either(
    //         fourPieceRight,
    //         fourPieceLeft,
    //         () -> {
    //           if ((Field2d.getInstance().getAlliance() == Alliance.Blue
    //                   && RobotOdometry.getInstance().getEstimatedPose().getY()
    //                       < FieldConstants.fieldWidth / 2.0)
    //               || (Field2d.getInstance().getAlliance() == Alliance.Red
    //                   && RobotOdometry.getInstance().getEstimatedPose().getY()
    //                       > FieldConstants.fieldWidth / 2.0)) {
    //             return true;
    //           } else {
    //             return false;
    //           }
    //         });
    // autoChooser.addDefaultOption("Auto Auto Selector", autoAutoSelector);

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

  // 4 Coral: J, K, L, A
  public Command getFourCoralLeftCloseCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    // PathPlannerPath collectCoralAfterJ2L;
    // PathPlannerPath scoreCoralL2L;
    // PathPlannerPath collectCoralAfterL2L;
    // PathPlannerPath scoreCoralK;
    // PathPlannerPath collectCoralAfterK;

    // try {
    //   collectCoralAfterJ2L = PathPlannerPath.fromPathFile("#2 Collect Coral After J 2L");
    //   scoreCoralL2L = PathPlannerPath.fromPathFile("#3 Score Coral L 2L");
    //   collectCoralAfterL2L = PathPlannerPath.fromPathFile("#4 Collect Coral After L 2L");
    //   scoreCoralK = PathPlannerPath.fromPathFile("#5 Score Coral K 3L");
    //   collectCoralAfterK = PathPlannerPath.fromPathFile("#6 Collect Coral After K 3L");
    // } catch (Exception e) {
    //   pathFileMissingAlert.setText(
    //       "Could not find the specified path file in getFourCoralLeftAutoCommand.");
    //   pathFileMissingAlert.set(true);

    //   return Commands.none();
    // }

    return Commands.sequence(
        Commands.runOnce(() -> timer.restart()),
        getScoreL4Command(
            drivetrain,
            vision,
            manipulator,
            elevator,
            () -> Field2d.getInstance().getNearestBranch(Side.RIGHT)),
        getCollectAndScoreCommand(drivetrain, manipulator, elevator, vision, Side.RIGHT, false),
        getCollectAndScoreCommand(drivetrain, manipulator, elevator, vision, Side.LEFT, false),
        Commands.either(
            getCollectAndScoreFourthCommand(
                drivetrain, manipulator, elevator, vision, Side.LEFT, false, true),
            Commands.none(), // if we can't collect the fourth coral, do nothing
            () -> (!timer.hasElapsed(12.0)) // only run if we have time left in auto
            ));
  }

  // 4 Coral: E, D, C, B
  public Command getFourCoralRightCloseCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    // PathPlannerPath collectCoralAfterE2R;
    // PathPlannerPath scoreCoralD2R;
    // PathPlannerPath collectCoralAfterD2R;
    // PathPlannerPath scoreCoralC;
    // PathPlannerPath collectCoralAfterC;

    // try {
    //   collectCoralAfterE2R = PathPlannerPath.fromPathFile("#2 Collect Coral After E 2R");
    //   scoreCoralD2R = PathPlannerPath.fromPathFile("#3 Score Coral D 2R");
    //   collectCoralAfterD2R = PathPlannerPath.fromPathFile("#4 Collect Coral After D 2R");
    //   scoreCoralC = PathPlannerPath.fromPathFile("#5 Score Coral C 3R");
    //   collectCoralAfterC = PathPlannerPath.fromPathFile("#6 Collect Coral After C 3R");
    // } catch (Exception e) {
    //   pathFileMissingAlert.setText(
    //       "Could not find the specified path file in getFourCoralRightAutoCommand.");
    //   pathFileMissingAlert.set(true);

    //   return Commands.none();
    // }

    return Commands.sequence(
        Commands.runOnce(() -> timer.restart()),
        getScoreL4Command(
            drivetrain,
            vision,
            manipulator,
            elevator,
            () -> Field2d.getInstance().getNearestBranch(Side.LEFT)),
        getCollectAndScoreCommand(drivetrain, manipulator, elevator, vision, Side.RIGHT, true),
        getCollectAndScoreCommand(drivetrain, manipulator, elevator, vision, Side.LEFT, true),
        Commands.either(
            getCollectAndScoreFourthCommand(
                drivetrain, manipulator, elevator, vision, Side.RIGHT, true, true),
            Commands.none(),
            () -> (!timer.hasElapsed(12.0))));
  }

  // 4 Coral: I, K, L, J
  public Command getFourCoralLeftFarCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    return Commands.sequence(
        Commands.runOnce(() -> timer.restart()),
        getScoreL4Command(
            drivetrain,
            vision,
            manipulator,
            elevator,
            () -> Field2d.getInstance().getNearestBranch(Side.LEFT)),
        getCollectAndScoreCommand(drivetrain, manipulator, elevator, vision, Side.LEFT, false),
        getCollectAndScoreCommand(drivetrain, manipulator, elevator, vision, Side.RIGHT, false),
        Commands.either(
            getCollectAndScoreFourthCommand(
                drivetrain, manipulator, elevator, vision, Side.RIGHT, false, false),
            Commands.none(),
            () -> (!timer.hasElapsed(11.0))));
  }

  // 4 Coral: F, D, C, E
  // maybe switch order of F and E in order to avoid collisions with other robots
  public Command getFourCoralRightFarCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    return Commands.sequence(
        Commands.runOnce(() -> timer.restart()),
        getScoreL4Command(
            drivetrain,
            vision,
            manipulator,
            elevator,
            () -> Field2d.getInstance().getNearestBranch(Side.RIGHT)),
        getCollectAndScoreCommand(drivetrain, manipulator, elevator, vision, Side.RIGHT, false),
        getCollectAndScoreCommand(drivetrain, manipulator, elevator, vision, Side.LEFT, false),
        Commands.either(
            getCollectAndScoreFourthCommand(
                drivetrain, manipulator, elevator, vision, Side.LEFT, true, false),
            Commands.none(),
            () -> (!timer.hasElapsed(11.0))));
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
        getScoreL4Command(
            drivetrain,
            vision,
            manipulator,
            elevator,
            () -> Field2d.getInstance().getNearestBranch(Side.LEFT)),
        CrossSubsystemsCommandsFactory.getCollectAlgaeCommand(
            drivetrain, manipulator, elevator, vision),
        AutoBuilder.followPath(backUpH1C));
  }

  public Command getOneCoralTwoAlgaeCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    PathPlannerPath backUpAfterFirstAlgae;
    PathPlannerPath backUpAfterSecondAlgae;
    try {
      backUpAfterFirstAlgae = PathPlannerPath.fromPathFile("Back Up After 1st Algae");
      backUpAfterSecondAlgae = PathPlannerPath.fromPathFile("Back Up After 2nd Algae");
    } catch (Exception e) {
      pathFileMissingAlert.setText(
          "Could not find the specified path file in getOneCoralTwoAlgaeCommand.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }

    // Use DRIVE-TO-PROCESSOR even though we are going to the barge so it uses the
    // bargeAndProcessorKp instead of running a normal Drive-To-Pose

    // we back up after the first algae so that our collect algae can get the correct "nearest
    // algae" pose

    //
    return Commands.sequence(
        getScoreL4BeforeAlgaeCommand(drivetrain, manipulator, elevator, vision, Side.LEFT),
        CrossSubsystemsCommandsFactory.getCollectAlgaeCommand(
            drivetrain, manipulator, elevator, vision),
        Commands.parallel(
            Commands.sequence(
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> elevator.goToPosition(ScoringHeight.BARGE), elevator)),
            new DriveToProcessor(
                drivetrain,
                () -> Field2d.getInstance().getRightBargePose(),
                manipulator::setReadyToScore,
                new Transform2d(
                    Units.inchesToMeters(0.5), Units.inchesToMeters(3), Rotation2d.fromDegrees(2)),
                3.0)),
        Commands.waitUntil(() -> elevator.isAtPosition(ScoringHeight.BARGE)),
        Commands.runOnce(manipulator::scoreAlgaeInBarge, manipulator),
        Commands.waitUntil(manipulator::scoredAlgae),
        Commands.parallel(
            AutoBuilder.followPath(backUpAfterFirstAlgae),
            elevator.getElevatorLowerAndResetCommand()),
        CrossSubsystemsCommandsFactory.getCollectAlgaeCommand(
            drivetrain, manipulator, elevator, vision),
        Commands.parallel(
            Commands.sequence(
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> elevator.goToPosition(ScoringHeight.BARGE), elevator)),
            new DriveToProcessor(
                drivetrain,
                () -> Field2d.getInstance().getCenterBargePose(),
                manipulator::setReadyToScore,
                new Transform2d(
                    Units.inchesToMeters(0.5), Units.inchesToMeters(3), Rotation2d.fromDegrees(2)),
                3.0)),
        Commands.waitUntil(() -> elevator.isAtPosition(ScoringHeight.BARGE)),
        Commands.runOnce(manipulator::scoreAlgaeInBarge, manipulator),
        Commands.waitUntil(manipulator::scoredAlgae),
        Commands.parallel(
            elevator.getElevatorLowerAndResetCommand(),
            AutoBuilder.followPath(backUpAfterSecondAlgae)));
  }

  public Command getTwoCoralBackLeftCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    PathPlannerPath collectCoralAfterG;
    PathPlannerPath driveToH;

    try {
      collectCoralAfterG = PathPlannerPath.fromPathFile("Collect Coral Left After G");
      driveToH = PathPlannerPath.fromPathFile("Drive To H Left");
    } catch (Exception e) {
      pathFileMissingAlert.setText(
          "Could not find the specified path file in getTwoCoralBackLeftCommand.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }

    return Commands.sequence(
        getScoreL4Command(
            drivetrain,
            vision,
            manipulator,
            elevator,
            () -> Field2d.getInstance().getNearestBranch(Side.LEFT)),
        Commands.parallel(
            AutoBuilder.followPath(collectCoralAfterG), elevator.getElevatorLowerAndResetCommand()),
        getCollectCoralCommand(manipulator),
        AutoBuilder.followPath(driveToH),
        getScoreL4Command(
            drivetrain,
            vision,
            manipulator,
            elevator,
            () -> Field2d.getInstance().getNearestBranch(Side.RIGHT)),
        elevator.getElevatorLowerAndResetCommand());
  }

  public Command getTwoCoralBackRightCommand(
      Drivetrain drivetrain, Vision vision, Manipulator manipulator, Elevator elevator) {
    PathPlannerPath collectCoralAfterH;
    PathPlannerPath driveToG;

    try {
      collectCoralAfterH = PathPlannerPath.fromPathFile("Collect Coral Right After H");
      driveToG = PathPlannerPath.fromPathFile("Drive To G Right");
    } catch (Exception e) {
      pathFileMissingAlert.setText(
          "Could not find the specified path file in getTwoCoralBackRightCommand.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }

    return Commands.sequence(
        getScoreL4Command(
            drivetrain,
            vision,
            manipulator,
            elevator,
            () -> Field2d.getInstance().getNearestBranch(Side.RIGHT)),
        Commands.parallel(
            AutoBuilder.followPath(collectCoralAfterH), elevator.getElevatorLowerAndResetCommand()),
        getCollectCoralCommand(manipulator),
        AutoBuilder.followPath(driveToG),
        getScoreL4Command(
            drivetrain,
            vision,
            manipulator,
            elevator,
            () -> Field2d.getInstance().getNearestBranch(Side.LEFT)),
        elevator.getElevatorLowerAndResetCommand());
  }

  // FIXME: maybe change this to robot relative? it doesn't really matter though since it's just one
  // direction
  private Command getBumpAndThreeCoralLeftCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    return Commands.sequence(
        Commands.run(() -> drivetrain.drive(-3.0, 0.0, 0.0, true, true), drivetrain)
            .withTimeout(1.0),
        getFourCoralLeftCloseCommand(drivetrain, vision, manipulator, elevator));
  }

  private Command getBumpAndThreeCoralRightCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    return Commands.sequence(
        Commands.run(() -> drivetrain.drive(-3.0, 0.0, 0.0, true, true), drivetrain)
            .withTimeout(1.0),
        getFourCoralRightCloseCommand(drivetrain, vision, manipulator, elevator));
  }

  private Command getScoreL4Command(
      Drivetrain drivetrain,
      Vision vision,
      Manipulator manipulator,
      Elevator elevator,
      Supplier<Pose2d> poseSupplier) {
    return Commands.sequence(
        Commands.parallel(
            Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2))),
            Commands.sequence(
                Commands.waitUntil(
                    manipulator::hasIndexedCoral), /* possibly add a fractional wait here */
                Commands.runOnce(
                    () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.L3), elevator)),
            new DriveToReef(
                drivetrain,
                poseSupplier,
                manipulator::setReadyToScore,
                elevator::setDistanceFromReef,
                new Transform2d(
                    DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
                    DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
                    Rotation2d.fromDegrees(DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
                4.0),
            Commands.sequence(
                Commands.waitUntil(elevator::canScoreFartherAway),
                Commands.runOnce(() -> elevator.goToPosition(ElevatorConstants.ScoringHeight.L4)))),
        Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ScoringHeight.L4)),
        Commands.parallel(
            Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3))),
            Commands.runOnce(manipulator::shootCoralFast, manipulator)),
        Commands.waitUntil(() -> !manipulator.coralIsInManipulator()),
        Commands.runOnce(() -> elevator.goToPosition(ScoringHeight.HARDSTOP), elevator));
  }

  private Command getScoreL4BeforeAlgaeCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision, Side side) {
    return Commands.sequence(
        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2))),
        Commands.parallel(
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
            Commands.sequence(
                Commands.runOnce(
                    () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.L3), elevator),
                Commands.waitUntil(elevator::canScoreFartherAway),
                Commands.runOnce(
                    () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.L4), elevator))),
        Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ScoringHeight.L4)),
        Commands.runOnce(manipulator::shootCoralFast, manipulator),
        Commands.waitUntil(() -> !manipulator.coralIsInManipulator()));
  }

  private Command getCollectAndScoreCommand(
      Drivetrain drivetrain,
      Manipulator manipulator,
      Elevator elevator,
      Vision vision,
      Side side,
      boolean rightCoralStation) {
    return Commands.sequence(
        new DriveToStation(
            drivetrain,
            manipulator,
            () ->
                (rightCoralStation
                    ? Field2d.getInstance().getRightCoralStation()
                    : Field2d.getInstance().getLeftCoralStation()),
            new Transform2d(
                Units.inchesToMeters(0.5), Units.inchesToMeters(1.0), Rotation2d.fromDegrees(2.0)),
            4.0),
        getCollectCoralCommand(manipulator),
        getScoreL4Command(
            drivetrain,
            vision,
            manipulator,
            elevator,
            () -> Field2d.getInstance().getNearestBranch(side)));
  }

  private Command getCollectAndScoreFourthCommand(
      Drivetrain drivetrain,
      Manipulator manipulator,
      Elevator elevator,
      Vision vision,
      Side side,
      boolean rightCoralStation,
      boolean closeAuto) {
    return Commands.sequence(
        new DriveToStation(
            drivetrain,
            manipulator,
            () ->
                (rightCoralStation
                    ? Field2d.getInstance().getRightCoralStation()
                    : Field2d.getInstance().getLeftCoralStation()),
            new Transform2d(
                Units.inchesToMeters(0.5), Units.inchesToMeters(1.0), Rotation2d.fromDegrees(2.0)),
            4.0),
        getCollectCoralCommand(manipulator),
        Commands.either(
            Commands.none(),
            getScoreL4Command(
                drivetrain,
                vision,
                manipulator,
                elevator,
                () -> Field2d.getInstance().getFourthAutoCoralPose(side, closeAuto)),
            () -> (timer.hasElapsed(13.0)) // FIXME: arbitrary tune this time
            ));
  }

  private Command getCollectCoralCommand(Manipulator manipulator) {
    return Commands.waitUntil(() -> manipulator.indexingCoral() || manipulator.hasIndexedCoral());
  }

  private Command createTuningAutoPath(
      String autoName, boolean measureDistance, Drivetrain drivetrain) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::captureInitialConditions),
        new PathPlannerAuto(autoName),
        Commands.runOnce(() -> drivetrain.captureFinalConditions(autoName, measureDistance)));
  }
}
