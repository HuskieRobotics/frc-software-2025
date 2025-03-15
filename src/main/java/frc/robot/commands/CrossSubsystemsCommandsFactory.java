package frc.robot.commands;

import static frc.robot.subsystems.elevator.ElevatorConstants.FAR_SCORING_DISTANCE;
import static frc.robot.subsystems.elevator.ElevatorConstants.MIN_FAR_SCORING_DISTANCE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.drivetrain.DrivetrainConstants;
import frc.lib.team3061.vision.Vision;
import frc.robot.Field2d;
import frc.robot.Field2d.AlgaePosition;
import frc.robot.Field2d.Side;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
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
      Vision vision) {

    oi.getScoreButton()
        .onTrue(
            Commands.either(
                getScoreCoralCommand(manipulator, elevator),
                getScoreWithAlgaeSelectedCommand(drivetrain, manipulator, elevator, vision),
                () ->
                    (OISelector.getOperatorInterface().getAlgaeBargeTrigger().getAsBoolean()
                        || OISelector.getOperatorInterface()
                            .getAlgaeProcessorTrigger()
                            .getAsBoolean()
                        || OISelector.getOperatorInterface()
                            .getAlgaeDropTrigger()
                            .getAsBoolean())));

    // FIXME: refactor this to have algae and coral options like prep button
    oi.getScoreButton()
        .onTrue(
            Commands.either(
                    getScoreL1Command(manipulator, elevator),
                    Commands.sequence(
                        Commands.either(
                            Commands.sequence(
                                Commands.runOnce(manipulator::removeAlgae, manipulator),
                                getScoreCoralCommand(manipulator, elevator),
                                Commands.runOnce(elevator::goBelowSelectedAlgaePosition, elevator),
                                Commands.runOnce(
                                    () -> vision.specifyCamerasToConsider(List.of(0, 2))),
                                Commands.waitUntil(elevator::isBelowSelectedAlgaePosition),
                                new DriveToReef(
                                    drivetrain,
                                    () -> Field2d.getInstance().getNearestBranch(Side.REMOVE_ALGAE),
                                    manipulator::setReadyToRemoveAlgae,
                                    elevator::setXFromReef,
                                    elevator::setYFromReef,
                                    elevator::setThetaFromReef,
                                    new Transform2d(
                                        DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
                                        DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
                                        Rotation2d.fromDegrees(
                                            DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
                                    0.5),
                                Commands.runOnce(elevator::goAboveSelectedAlgaePosition, elevator),
                                Commands.runOnce(
                                    () -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3))),
                                Commands.waitUntil(elevator::isAboveSelectedAlgaePosition),
                                Commands.waitSeconds(0.5),
                                Commands.runOnce(manipulator::algaeIsRemoved, manipulator)),
                            getScoreCoralCommand(manipulator, elevator),
                            elevator::isAlgaePositionSelected),
                        Commands.deadline(
                            // run TeleopSwerve to allow driver to move away from reef while
                            // elevator is lowering
                            elevator.getElevatorLowerAndResetCommand(),
                            new TeleopSwerve(
                                drivetrain,
                                OISelector.getOperatorInterface()::getTranslateX,
                                OISelector.getOperatorInterface()::getTranslateY,
                                OISelector.getOperatorInterface()::getRotate))),
                    () -> OISelector.getOperatorInterface().getLevel1Trigger().getAsBoolean())
                .withName("score coral"));

    oi.getPrepToScoreButton()
        .onTrue(
            Commands.either(
                    getPrepCoralCommand(drivetrain, manipulator, elevator, vision),
                    getPrepAlgaeCommand(drivetrain, manipulator, elevator, vision, oi),
                    manipulator::hasCoral)
                .withName("prep to score"));

    oi.getInterruptAll().onTrue(getInterruptAllCommand(manipulator, elevator, drivetrain, oi));

    oi.getOverrideDriveToPoseButton().onTrue(getDriveToPoseOverrideCommand(drivetrain, oi));
  }

  private static Command getScoreCoralCommand(Manipulator manipulator, Elevator elevator) {
    return Commands.either(
        Commands.either(
            getScoreOneCoralAwayCommand(manipulator, elevator, ScoringHeight.MAX_L2),
            getScoreOneCoralAwayCommand(manipulator, elevator, ScoringHeight.MAX_L3),
            () -> OISelector.getOperatorInterface().getLevel2Trigger().getAsBoolean()),
        getScoreCoralCloseCommand(manipulator, elevator),
        () ->
            !(OISelector.getOperatorInterface().getLevel1Trigger().getAsBoolean()
                || OISelector.getOperatorInterface().getLevel4Trigger().getAsBoolean()));
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

  private static Command getScoreOneCoralAwayCommand(
      Manipulator manipulator, Elevator elevator, ScoringHeight branch) {
    return Commands.sequence(
        Commands.either(
            Commands.sequence(
                Commands.runOnce(() -> elevator.goToPosition(branch)),
                Commands.waitUntil(() -> elevator.isAtPosition(branch)),
                Commands.runOnce(manipulator::shootCoralSlow, manipulator)),
            Commands.runOnce(manipulator::shootCoralFast, manipulator),
            () ->
                Math.abs(elevator.getXFromReef()) > MIN_FAR_SCORING_DISTANCE
                    && Math.abs(elevator.getXFromReef()) < FAR_SCORING_DISTANCE),
        Commands.waitUntil(() -> !manipulator.hasCoral()),
        Commands.runOnce(() -> elevator.setXFromReef(100.0)));
  }

  private static Command getScoreCoralCloseCommand(Manipulator manipulator, Elevator elevator) {
    return Commands.sequence(
        Commands.runOnce(manipulator::shootCoralFast, manipulator),
        Commands.waitUntil(() -> !manipulator.hasCoral()),
        Commands.runOnce(() -> elevator.setXFromReef(100.0)));
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
      Manipulator manipulator, Elevator elevator, Drivetrain drivetrain, OperatorInterface oi) {
    return Commands.parallel(
            Commands.sequence(
                Commands.runOnce(manipulator::shootCoralFast, manipulator),
                Commands.runOnce(
                    () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.HARDSTOP),
                    elevator),
                Commands.runOnce(manipulator::resetStateMachine, manipulator)),
            new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate))
        .withName("interrupt all");
  }

  private static Command getDriveToPoseOverrideCommand(
      Drivetrain drivetrain, OperatorInterface oi) {
    return new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)
        .withName("Override driveToPose");
  }

  // go to selected reef spot or don't move (depending on sign of x difference)
  // this logic will get handled in drivetoreef
  // red flash for a second if we can't move
  private static Command getPrepCoralCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    return Commands.sequence(
        Commands.sequence(
            Commands.waitUntil(manipulator::hasIndexedCoral),
            Commands.parallel(
                Commands.sequence(
                    Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2))),
                    new DriveToReef(
                        drivetrain,
                        () -> Field2d.getInstance().getSelectedBranch(),
                        manipulator::setReadyToScore,
                        elevator::setXFromReef,
                        elevator::setYFromReef,
                        elevator::setThetaFromReef,
                        new Transform2d(
                            DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
                            DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
                            Rotation2d.fromDegrees(
                                DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
                        5.0),
                    Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3)))),
                Commands.runOnce(elevator::goToSelectedPosition, elevator))));
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
        manipulator::hasAlgae);
  }

  private static Command getPrepToScoreAlgaeCommand(
      Drivetrain drivetrain,
      Manipulator manipulator,
      Elevator elevator,
      Vision vision,
      OperatorInterface oi) {
    if (OISelector.getOperatorInterface().getAlgaeBargeTrigger().getAsBoolean()) {
      return Commands.parallel(
          Commands.runOnce(
              () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.BARGE), elevator),
          new DriveToBarge(
              drivetrain,
              () -> Field2d.getInstance().getBargePose(),
              manipulator::setReadyToScore,
              manipulator::shotAlgae,
              oi::getTranslateY));
    } else if (OISelector.getOperatorInterface().getAlgaeProcessorTrigger().getAsBoolean()) {
      return Commands.parallel(
          Commands.runOnce(
              () -> elevator.goToPosition(ElevatorConstants.ScoringHeight.PROCESSOR), elevator),
          new DriveToProcessor(
              drivetrain,
              () -> Field2d.getInstance().getNearestProcessor(),
              manipulator::setReadyToScore,
              new Transform2d(
                  DrivetrainConstants.DRIVE_TO_PROCESSOR_X_TOLERANCE,
                  DrivetrainConstants.DRIVE_TO_PROCESSOR_Y_TOLERANCE,
                  Rotation2d.fromDegrees(
                      DrivetrainConstants.DRIVE_TO_PROCESSOR_THETA_TOLERANCE_DEG)),
              3.0));
    }

    return Commands.none();
  }

  private static Command getCollectAlgaeCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    AlgaePosition nearestAlgae = Field2d.getInstance().getNearestAlgae();
    Pose2d nearestAlgaePose = nearestAlgae.pose;
    boolean isHighAlgae = nearestAlgae.isHigh;

    return Commands.parallel(
        Commands.either(
            Commands.runOnce(() -> elevator.goToPosition(ScoringHeight.HIGH_ALGAE), elevator),
            Commands.runOnce(() -> elevator.goToPosition(ScoringHeight.LOW_ALGAE), elevator),
            () -> isHighAlgae),
        Commands.sequence(
            Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2))),
            new DriveToReef(
                drivetrain,
                () -> nearestAlgaePose,
                manipulator::setReadyToScore,
                elevator::setXFromReef,
                elevator::setYFromReef,
                elevator::setThetaFromReef,
                new Transform2d(
                    DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
                    DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
                    Rotation2d.fromDegrees(DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
                3.0),
            Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3)))),
        Commands.sequence(
            Commands.runOnce(manipulator::collectAlgae, manipulator),
            Commands.waitUntil(manipulator::hasAlgae)));
  }

  private static Command getScoreWithAlgaeSelectedCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    return Commands.either(
        getScoreAlgaeCommand(manipulator, elevator),
        getScoreCoralAndCollectAlgaeCommand(drivetrain, manipulator, elevator, vision),
        manipulator::hasAlgae);
  }

  private static Command getScoreAlgaeCommand(Manipulator manipulator, Elevator elevator) {
    return Commands.sequence(
        Commands.runOnce(manipulator::shootAlgae, manipulator),
        Commands.waitUntil(() -> !manipulator.hasAlgae()));
  }

  private static Command getScoreCoralAndCollectAlgaeCommand(
      Drivetrain drivetrain, Manipulator manipulator, Elevator elevator, Vision vision) {
    // FIXME: adapt for new algae height system, use from prep to score commands?
    return Commands.sequence(
        Commands.runOnce(manipulator::removeAlgae, manipulator),
        getScoreCoralCommand(manipulator, elevator),
        Commands.runOnce(elevator::goBelowSelectedAlgaePosition, elevator),
        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2))),
        Commands.waitUntil(elevator::isBelowSelectedAlgaePosition),
        new DriveToReef(
            drivetrain,
            () -> Field2d.getInstance().getNearestBranch(Side.REMOVE_ALGAE),
            manipulator::setReadyToRemoveAlgae,
            elevator::setXFromReef,
            elevator::setYFromReef,
            elevator::setThetaFromReef,
            new Transform2d(
                DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE,
                DrivetrainConstants.DRIVE_TO_REEF_Y_TOLERANCE,
                Rotation2d.fromDegrees(DrivetrainConstants.DRIVE_TO_REEF_THETA_TOLERANCE_DEG)),
            0.5),
        Commands.runOnce(elevator::goAboveSelectedAlgaePosition, elevator),
        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3))),
        Commands.waitUntil(elevator::isAboveSelectedAlgaePosition),
        Commands.waitSeconds(0.5),
        Commands.runOnce(manipulator::algaeIsRemoved, manipulator));
  }
}
