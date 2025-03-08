package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.vision.Vision;
import frc.robot.Field2d;
import frc.robot.Field2d.Side;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
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
    oi.getScoreCoralButton()
        .onTrue(
            Commands.either(
                    Commands.sequence(
                        getScoreL1Command(manipulator, elevator),
                        Commands.waitUntil(manipulator::isWaitingForCoral)),
                    Commands.sequence(
                        Commands.either(
                            Commands.parallel(
                                Commands.runOnce(manipulator::removeAlgae),
                                Commands.sequence(
                                    getScoreCoralCommand(manipulator),
                                    Commands.runOnce(
                                        elevator::goBelowSelectedAlgaePosition, elevator),
                                    Commands.runOnce(
                                        () -> vision.specifyCamerasToConsider(List.of(0, 2))),
                                    Commands.waitUntil(elevator::isBelowSelectedAlgaePosition),
                                    new DriveToReef(
                                        drivetrain,
                                        () ->
                                            Field2d.getInstance()
                                                .getNearestBranch(Side.REMOVE_ALGAE),
                                        manipulator::setReadyToRemoveAlgae,
                                        elevator::getDistanceFromReef,
                                        new Transform2d(
                                            Units.inchesToMeters(0.5),
                                            Units.inchesToMeters(0.5),
                                            Rotation2d.fromDegrees(2.0)),
                                        0.5),
                                    Commands.runOnce(
                                        elevator::goAboveSelectedAlgaePosition, elevator),
                                    Commands.runOnce(
                                        () -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3))),
                                    Commands.waitUntil(elevator::isAboveSelectedAlgaePosition),
                                    Commands.waitSeconds(0.5),
                                    Commands.runOnce(manipulator::algaeIsRemoved))),
                            getScoreCoralCommand(manipulator),
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

    oi.getDescoreAlgaeAfterAutoButton()
        .onTrue(
            AutonomousCommandFactory.getInstance()
                .getDescoreAlgaeCommand(drivetrain, vision, manipulator, elevator));

    // drive to left branch of nearest reef face
    oi.getPrepToScoreCoralLeftButton()
        .onTrue(
            Commands.sequence(
                    Commands.waitUntil(manipulator::hasIndexedCoral),
                    Commands.deadline(
                        Commands.sequence(
                            Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2))),
                            new DriveToReef(
                                drivetrain,
                                () -> Field2d.getInstance().getNearestBranch(Side.LEFT),
                                manipulator::setReadyToScore,
                                elevator::getDistanceFromReef,
                                new Transform2d(
                                    Units.inchesToMeters(0.5),
                                    Units.inchesToMeters(0.5),
                                    Rotation2d.fromDegrees(2.0)),
                                3.0),
                            Commands.runOnce(
                                () -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3)))),
                        Commands.run(elevator::goToSelectedPosition, elevator)))
                .withName("drive to nearest left branch"));

    // drive to right branch of nearest reef face
    oi.getPrepToScoreCoralRightButton()
        .onTrue(
            Commands.sequence(
                    Commands.waitUntil(manipulator::hasIndexedCoral),
                    Commands.deadline(
                        Commands.sequence(
                            Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2))),
                            new DriveToReef(
                                drivetrain,
                                () -> Field2d.getInstance().getNearestBranch(Side.RIGHT),
                                manipulator::setReadyToScore,
                                elevator::getDistanceFromReef,
                                new Transform2d(
                                    Units.inchesToMeters(0.5),
                                    Units.inchesToMeters(0.5),
                                    Rotation2d.fromDegrees(2.0)),
                                3.0),
                            Commands.runOnce(
                                () -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3)))),
                        Commands.run(elevator::goToSelectedPosition, elevator)))
                .withName("drive to nearest right branch"));

    oi.getInterruptAll().onTrue(getInterruptAllCommand(manipulator, elevator, drivetrain, oi));

    oi.getDriveToPoseOverrideButton().onTrue(getDriveToPoseOverrideCommand(drivetrain, oi));
  }

  private static Command getScoreCoralCommand(Manipulator manipulator) {
    return Commands.sequence(
        Commands.runOnce(manipulator::shootCoral, manipulator),
        Commands.waitUntil(() -> !manipulator.hasCoral()));
  }

  private static Command getScoreL1Command(Manipulator manipulator, Elevator elevator) {
    return Commands.sequence(
        Commands.runOnce(() -> elevator.goToPosition(ElevatorConstants.ReefBranch.L1)),
        Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ReefBranch.L1)),
        Commands.runOnce(manipulator::shootCoral, manipulator),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> elevator.goToPosition(ElevatorConstants.ReefBranch.ABOVE_L1)),
        Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ReefBranch.ABOVE_L1)),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> elevator.goToPosition(ElevatorConstants.ReefBranch.HARDSTOP)),
        Commands.waitUntil(() -> elevator.isAtPosition(ElevatorConstants.ReefBranch.HARDSTOP)));
  }

  // interrupt all commands by running a command that requires every subsystem. This is used to
  // recover to a known state if the robot becomes "stuck" in a command.
  // "run all wheels backwards and bring elevator and carriage back to initial configuration"

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
                Commands.runOnce(manipulator::shootCoral, manipulator),
                Commands.runOnce(
                    () -> elevator.goToPosition(ElevatorConstants.ReefBranch.HARDSTOP), elevator),
                Commands.runOnce(manipulator::resetStateMachine, manipulator)),
            new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate))
        .withName("interrupt all");
  }

  private static Command getDriveToPoseOverrideCommand(
      Drivetrain drivetrain, OperatorInterface oi) {
    return new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)
        .withName("Override driveToPose");
  }
}
