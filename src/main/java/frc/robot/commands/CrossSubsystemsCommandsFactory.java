package frc.robot.commands;

import static frc.robot.subsystems.elevator.ElevatorConstants.FAR_SCORING_DISTANCE;
import static frc.robot.subsystems.elevator.ElevatorConstants.MIN_FAR_SCORING_DISTANCE;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.drivetrain.DrivetrainConstants;
import frc.lib.team3061.vision.Vision;
import frc.robot.Field2d;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ScoringHeight;
import frc.robot.subsystems.manipulator.Manipulator;
import java.util.List;

public class CrossSubsystemsCommandsFactory {

  private CrossSubsystemsCommandsFactory() {}

  public static void registerCommands(
      OperatorInterface oi,
      Drivetrain drivetrain,
      Elevator elevator,
      Manipulator manipulator,
      Climber climber,
      Vision vision) {

    oi.getScoreButton()
        .onTrue(
            Commands.sequence(
                    Commands.either(
                        getScoreWithAlgaeSelectedCommand(drivetrain, manipulator, elevator, vision),
                        Commands.sequence(
                            getScoreCoralCommand(manipulator, elevator),
                            Commands.deadline(
                                getElevatorLowerAndResetCommand(elevator, manipulator),
                                new TeleopSwerve(
                                    drivetrain,
                                    OISelector.getOperatorInterface()::getTranslateX,
                                    OISelector.getOperatorInterface()::getTranslateY,
                                    OISelector.getOperatorInterface()::getRotate))),
                        () ->
                            (OISelector.getOperatorInterface().getAlgaeBargeTrigger().getAsBoolean()
                                || OISelector.getOperatorInterface()
                                    .getAlgaeProcessorTrigger()
                                    .getAsBoolean()
                                || OISelector.getOperatorInterface()
                                    .getAlgaeDropTrigger()
                                    .getAsBoolean())),
                    Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3))))
                .withName("score"));

    oi.getPrepToScoreButton()
        .onTrue(
            Commands.either(
                    Commands.sequence(
                        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2))),
                        Commands.either(
                            Commands.sequence(
                                getPrepAndAutoScoreCoralCommand(
                                    drivetrain, manipulator, elevator, vision),
                                Commands.runOnce(
                                    () -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3)))),
                            getPrepCoralCommand(drivetrain, manipulator, elevator, vision),
                            () ->
                                OISelector.getOperatorInterface()
                                    .getEnableAutoScoringTrigger()
                                    .getAsBoolean())),
                    getPrepAlgaeCommand(drivetrain, manipulator, elevator, vision, oi),
                    manipulator::hasIndexedCoral)
                .withName("prep to score"));

    oi.getDriveToNearestCoralStationButton()
        .onTrue(
            new DriveToStation(
                    drivetrain,
                    manipulator,
                    () -> Field2d.getInstance().getNearestCoralStation(),
                    new Transform2d(
                        Units.inchesToMeters(0.5),
                        Units.inchesToMeters(1.0),
                        Rotation2d.fromDegrees(2.0)),
                    3.0)
                .withName("drive to nearest coral station"));

    oi.getInterruptAll()
        .onTrue(getInterruptAllCommand(manipulator, elevator, drivetrain, climber, vision, oi));

    oi.getOverrideDriveToPoseButton().onTrue(getDriveToPoseOverrideCommand(drivetrain, oi));
  }

  private static Command getScoreCoralCommand(Manipulator manipulator, Elevator elevator) {
    return Commands.sequence(
        Commands.either(
            Commands.either(
                getScoreL2L3Command(manipulator, elevator, ScoringHeight.MAX_L2),
                getScoreL2L3Command(manipulator, elevator, ScoringHeight.MAX_L3),
                () -> OISelector.getOperatorInterface().getLevel2Trigger().getAsBoolean()),
            getScoreCoralCloseCommand(manipulator, elevator),
            () ->
                !(OISelector.getOperatorInterface().getLevel1Trigger().getAsBoolean()
                    || OISelector.getOperatorInterface().getLevel4Trigger().getAsBoolean())),
        Commands.runOnce(() -> elevator.setXFromReef(100.0)));
  }

  private static Command getScoreL1Command(Manipulator manipulator, Elevator elevator) {
    return Commands.sequence(
        Commands.runOnce(() -> elevator.goToPosition(ElevatorConstants.ScoringHeight.L1), elevator),
        Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ScoringHeight.L1)),
        Commands.runOnce(manipulator::shootCoralFast, manipulator),
        Commands.waitSeconds(0.25),
        Commands.runOnce(
            () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.ABOVE_L1), elevator),
        Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ScoringHeight.ABOVE_L1)),
        Commands.runOnce(
            () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.HARDSTOP), elevator));
  }

  private static Command getScoreL2L3Command(
      Manipulator manipulator, Elevator elevator, ScoringHeight branch) {
    return Commands.sequence(
        Commands.either(
            Commands.sequence(
                Commands.runOnce(() -> elevator.goToPosition(branch)),
                Commands.waitUntil(() -> elevator.isAtPosition(branch)),
                Commands.runOnce(manipulator::shootCoralFast, manipulator)),
            Commands.runOnce(manipulator::shootCoralSlow, manipulator),
            () ->
                Math.abs(elevator.getXFromReef()) > MIN_FAR_SCORING_DISTANCE
                    && Math.abs(elevator.getXFromReef()) < FAR_SCORING_DISTANCE),
        Commands.waitUntil(() -> !manipulator.coralIsInManipulator()));
  }

  private static Command getScoreCoralCloseCommand(Manipulator manipulator, Elevator elevator) {
    return Commands.either(
        getScoreL1Command(manipulator, elevator),
        Commands.sequence(
            Commands.runOnce(elevator::goToSelectedPosition, elevator),
            Commands.waitUntil(elevator::isAtSelectedPosition),
            Commands.runOnce(manipulator::shootCoralFast, manipulator),
            Commands.waitUntil(() -> !manipulator.coralIsInManipulator())),
        () -> OISelector.getOperatorInterface().getLevel1Trigger().getAsBoolean());
  }

  private static Command getPrepAndAutoScoreCoralCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    return Commands.sequence(
            Commands.waitUntil(manipulator::hasIndexedCoral),
            Commands.either(
                Commands.sequence(
                    getAutoScoreL4Command(drivetrain, manipulator, elevator, vision),
                    Commands.deadline(
                        getElevatorLowerAndResetCommand(elevator, manipulator),
                        new TeleopSwerve(
                            drivetrain,
                            OISelector.getOperatorInterface()::getTranslateX,
                            OISelector.getOperatorInterface()::getTranslateY,
                            OISelector.getOperatorInterface()::getRotate))),
                Commands.either(
                    /* either auto score l2 l3 or just prep the l1 */
                    Commands.sequence(
                        getAutoScoreL2L3Command(drivetrain, manipulator, elevator, vision),
                        Commands.deadline(
                            getElevatorLowerAndResetCommand(elevator, manipulator),
                            new TeleopSwerve(
                                drivetrain,
                                OISelector.getOperatorInterface()::getTranslateX,
                                OISelector.getOperatorInterface()::getTranslateY,
                                OISelector.getOperatorInterface()::getRotate))),
                    getPrepCoralCommand(drivetrain, manipulator, elevator, vision),
                    () -> !OISelector.getOperatorInterface().getLevel1Trigger().getAsBoolean()),
                () -> OISelector.getOperatorInterface().getLevel4Trigger().getAsBoolean()))
        .withName("prep and auto score");
  }

  /*
   * 1. Shoot coral
   * 2. Bring elevator down to hardstop (can further adjust with manual override if necessary)
   * 3. Reset manipulator state machine
   * 4. Interrupt drive to pose (run TeleopSwerve)
   */
  /*
   * If we want to truly reset the state machine, we can add a condition within the WAITING state that does not allow us
   * to pass to the next state until 1-2 seconds after we interrupt all. This would stop the state from transitioning even if the
   * IRs say that we should.
   */
  private static Command getInterruptAllCommand(
      Manipulator manipulator,
      Elevator elevator,
      Drivetrain drivetrain,
      Climber climber,
      Vision vision,
      OperatorInterface oi) {
    return Commands.parallel(
            Commands.sequence(
                Commands.runOnce(manipulator::shootCoralFast, manipulator),
                Commands.runOnce(
                    () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.HARDSTOP),
                    elevator),
                Commands.runOnce(manipulator::resetStateMachine, manipulator),
                Commands.runOnce(climber::stop, climber)),
            new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate),
            Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3))))
        .withName("interrupt all");
  }

  private static Command getDriveToPoseOverrideCommand(
      Drivetrain drivetrain, OperatorInterface oi) {
    return new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)
        .withName("Override driveToPose");
  }

  private static Command getPrepCoralCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    return Commands.sequence(
        Commands.waitUntil(manipulator::hasIndexedCoral),
        Commands.either(
            Commands.sequence(
                Commands.runOnce(elevator::goToSelectedPosition, elevator),
                Commands.waitUntil(() -> elevator.isAtPosition(ScoringHeight.L1))),
            Commands.either(
                getPrepL4CoralCommand(drivetrain, manipulator, elevator, vision),
                getPrepL2L3CoralCommand(drivetrain, manipulator, elevator, vision),
                () -> OISelector.getOperatorInterface().getLevel4Trigger().getAsBoolean()),
            () -> OISelector.getOperatorInterface().getLevel1Trigger().getAsBoolean()));
  }

  private static Command getPrepAlgaeCommand(
      Drivetrain drivetrain,
      Manipulator manipulator,
      Elevator elevator,
      Vision vision,
      OperatorInterface oi) {
    return Commands.either(
        getPrepToScoreAlgaeCommand(drivetrain, manipulator, elevator, vision, oi),
        getCollectAlgaeCommand(drivetrain, manipulator, elevator, vision),
        manipulator::hasIndexedAlgae);
  }

  private static Command getPrepToScoreAlgaeCommand(
      Drivetrain drivetrain,
      Manipulator manipulator,
      Elevator elevator,
      Vision vision,
      OperatorInterface oi) {
    return Commands.either(
        getPrepAlgaeBargeCommand(drivetrain, manipulator, elevator, oi),
        Commands.either(
            Commands.parallel(
                Commands.runOnce(
                    () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.PROCESSOR),
                    elevator),
                new DriveToProcessor(
                    drivetrain,
                    () -> Field2d.getInstance().getNearestProcessor(),
                    manipulator::setReadyToScore,
                    new Transform2d(
                        DrivetrainConstants.DRIVE_TO_PROCESSOR_X_TOLERANCE,
                        DrivetrainConstants.DRIVE_TO_PROCESSOR_Y_TOLERANCE,
                        Rotation2d.fromDegrees(
                            DrivetrainConstants.DRIVE_TO_PROCESSOR_THETA_TOLERANCE_DEG)),
                    3.0)),
            Commands.none(),
            () -> OISelector.getOperatorInterface().getAlgaeProcessorTrigger().getAsBoolean()),
        () -> OISelector.getOperatorInterface().getAlgaeBargeTrigger().getAsBoolean());
  }

  public static Command getPrepAlgaeBargeCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, OperatorInterface oi) {
    // check if we are short of barge or far of barge
    // if we are short of barge, check if we are within 2 feet. if we are, then put elevator up
    // first. uf not, do in simultaneous

    // FIXME: ask ian about this; just canceling the drive to barge for now
    // being under the barge at our target pose would cause us to still probably
    // raise directly through the barge on the way up.
    // would need to go backwards and then forwards again. May not be worth due to how quick the
    // driver could manually do that if we just cancel.
    // if we are far of barge, then drive backwards first and then raise the elevator up
    return Commands.either(
        Commands.either(
            Commands.parallel(
                Commands.runOnce(
                    () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.BARGE), elevator),
                new DriveToBarge(
                    drivetrain,
                    () -> Field2d.getInstance().getCenterBargePose(),
                    manipulator::setReadyToScore,
                    // FIXME: make these constants
                    new Transform2d(Units.inchesToMeters(1.0), 20.0, Rotation2d.fromDegrees(5.0)),
                    oi::getTranslateY)),
            Commands.sequence(
                Commands.runOnce(
                    () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.BARGE), elevator),
                Commands.waitUntil(
                    () -> elevator.isAtPosition(ElevatorConstants.ScoringHeight.BARGE)),
                new DriveToBarge(
                    drivetrain,
                    () -> Field2d.getInstance().getCenterBargePose(),
                    manipulator::setReadyToScore,
                    new Transform2d(Units.inchesToMeters(1.0), 20.0, Rotation2d.fromDegrees(5.0)),
                    oi::getTranslateY)),
            () -> Field2d.getInstance().isFarFromBarge()),
        Commands.runOnce(() -> drivetrain.setDriveToPoseCanceled(true)),
        () -> Field2d.getInstance().isShortOfBarge());
  }

  public static Command getCollectAlgaeCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    // separate collecting algae after scoring or just going to collect algae
    return Commands.sequence(
        Commands.runOnce(() -> elevator.goBelowNearestAlgae(), elevator),
        Commands.waitUntil(elevator::isBelowNearestAlgae),
        Commands.parallel(
            Commands.runOnce(manipulator::collectAlgae, manipulator),
            Commands.sequence(
                Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2))),
                new DriveToReef(
                    drivetrain,
                    () -> Field2d.getInstance().getNearestAlgae().pose,
                    manipulator::setReadyToScore,
                    elevator::setDistanceFromReef,
                    new Transform2d(
                        DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
                        DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
                        Rotation2d.fromDegrees(
                            DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
                    3.0),
                Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3))))),
        Commands.runOnce(() -> manipulator.setReadyToScore(false), manipulator),
        Commands.runOnce(() -> elevator.goToNearestAlgae(), elevator),
        Commands.waitUntil(manipulator::doneCollectingAlgae),
        Commands.either(
            getLeaveReefZoneCommand(drivetrain, elevator),
            Commands.none(),
            () -> OISelector.getOperatorInterface().getAlgaeProcessorTrigger().getAsBoolean()));
  }

  private static Command getScoreWithAlgaeSelectedCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    return Commands.either(
        Commands.sequence(
            getScoreAlgaeCommand(drivetrain, manipulator, elevator),
            Commands.deadline(
                getElevatorLowerAndResetCommand(elevator, manipulator),
                new TeleopSwerve(
                    drivetrain,
                    OISelector.getOperatorInterface()::getTranslateX,
                    OISelector.getOperatorInterface()::getTranslateY,
                    OISelector.getOperatorInterface()::getRotate))),
        getScoreCoralAndCollectAlgaeCommand(drivetrain, manipulator, elevator, vision),
        manipulator::hasIndexedAlgae);
  }

  private static Command getScoreAlgaeCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator) {
    return Commands.sequence(
        Commands.either(
            Commands.runOnce(manipulator::scoreAlgaeInBarge),
            Commands.either(
                Commands.runOnce(manipulator::scoreAlgaeInProcessor),
                Commands.runOnce(manipulator::dropAlgae),
                () -> OISelector.getOperatorInterface().getAlgaeProcessorTrigger().getAsBoolean()),
            () -> OISelector.getOperatorInterface().getAlgaeBargeTrigger().getAsBoolean()),
        Commands.waitUntil(() -> manipulator.scoredAlgae()));
  }

  private static Command getScoreCoralAndCollectAlgaeCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    return Commands.sequence(
        getScoreCoralCommand(manipulator, elevator),
        getCollectAlgaeCommand(drivetrain, manipulator, elevator, vision));
  }

  private static Command getLeaveReefZoneCommand(Drivetrain drivetrain, Elevator elevator) {
    return Commands.deadline(
        Commands.sequence(
            Commands.waitUntil(() -> Field2d.getInstance().isOutsideOfReefZone()),
            Commands.runOnce(
                () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.PROCESSOR), elevator)),
        new TeleopSwerve(
            drivetrain,
            OISelector.getOperatorInterface()::getTranslateX,
            OISelector.getOperatorInterface()::getTranslateY,
            OISelector.getOperatorInterface()::getRotate));
  }

  private static Command getAutoScoreL4Command(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    return Commands.sequence(
        getPrepL4CoralCommand(drivetrain, manipulator, elevator, vision),
        /* possibly add a fractional wait here */
        Commands.either(
            getScoreCoralAndCollectAlgaeCommand(drivetrain, manipulator, elevator, vision),
            Commands.sequence(
                Commands.runOnce(manipulator::shootCoralFast, manipulator),
                Commands.waitUntil(() -> !manipulator.coralIsInManipulator())),
            () ->
                (OISelector.getOperatorInterface().getAlgaeBargeTrigger().getAsBoolean()
                    || OISelector.getOperatorInterface().getAlgaeProcessorTrigger().getAsBoolean()
                    || OISelector.getOperatorInterface().getAlgaeDropTrigger().getAsBoolean())));
  }

  private static Command getAutoScoreL2L3Command(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    return Commands.sequence(
        getPrepL2L3CoralCommand(drivetrain, manipulator, elevator, vision),
        /* possibly add a fractional wait here */
        Commands.either(
            getScoreCoralAndCollectAlgaeCommand(drivetrain, manipulator, elevator, vision),
            Commands.sequence(
                Commands.runOnce(manipulator::shootCoralSlow, manipulator),
                Commands.waitUntil(() -> !manipulator.coralIsInManipulator())),
            () ->
                (OISelector.getOperatorInterface().getAlgaeBargeTrigger().getAsBoolean()
                    || OISelector.getOperatorInterface().getAlgaeProcessorTrigger().getAsBoolean()
                    || OISelector.getOperatorInterface().getAlgaeDropTrigger().getAsBoolean())));
  }

  private static Command getPrepL4CoralCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    return Commands.sequence(
        Commands.parallel(
            Commands.runOnce(() -> elevator.goToPosition(ScoringHeight.L3), elevator),
            new DriveToReef(
                drivetrain,
                () -> Field2d.getInstance().getSelectedBranch(),
                manipulator::setReadyToScore,
                elevator::setDistanceFromReef,
                new Transform2d(
                    DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
                    DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
                    Rotation2d.fromDegrees(DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
                5.0)),
        Commands.runOnce(() -> elevator.goToPosition(ScoringHeight.L4), elevator),
        Commands.waitUntil(elevator::isAtSelectedPosition));
  }

  private static Command getPrepL2L3CoralCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    return Commands.sequence(
        Commands.parallel(
            Commands.runOnce(elevator::goToSelectedPosition, elevator),
            new DriveToReef(
                drivetrain,
                () -> Field2d.getInstance().getSelectedBranch(),
                manipulator::setReadyToScore,
                elevator::setDistanceFromReef,
                new Transform2d(
                    DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
                    DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
                    Rotation2d.fromDegrees(DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
                5.0)),
        Commands.waitUntil(elevator::isAtSelectedPosition));
  }

  private static Command getElevatorLowerAndResetCommand(
      Elevator elevator, Manipulator manipulator) {
    return Commands.sequence(
        elevator.getElevatorLowerAndResetCommand(),
        Commands.runOnce(() -> manipulator.setReadyForCoral(true), manipulator));
  }
}
