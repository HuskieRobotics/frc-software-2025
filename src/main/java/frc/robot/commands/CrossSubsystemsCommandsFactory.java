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
            Commands.sequence(
                    Commands.either(
                        Commands.parallel(
                            Commands.runOnce(manipulator::removeAlgae),
                            Commands.sequence(
                                getScoreCoralCommand(manipulator),
                                Commands.runOnce(elevator::goBelowSelectedAlgaePosition, elevator),
                                Commands.waitUntil(elevator::isBelowSelectedAlgaePosition),
                                new DriveToPose(
                                    drivetrain,
                                    () -> Field2d.getInstance().getNearestBranch(Side.REMOVE_ALGAE),
                                    new Transform2d(
                                        Units.inchesToMeters(1.0),
                                        Units.inchesToMeters(1.0),
                                        Rotation2d.fromDegrees(2.0))),
                                Commands.runOnce(elevator::goAboveSelectedAlgaePosition, elevator),
                                Commands.waitUntil(elevator::isAboveSelectedAlgaePosition),
                                Commands.waitSeconds(1.0),
                                Commands.runOnce(manipulator::algaeIsRemoved))),
                        getScoreCoralCommand(manipulator),
                        elevator::isAlgaePositionSelected),
                    Commands.runOnce(
                        () -> elevator.goToPosition(ElevatorConstants.ReefBranch.HARDSTOP),
                        elevator))
                .withName("score coral"));

    // drive to left branch of nearest reef face
    oi.getPrepToScoreCoralLeftButton()
        .onTrue(
            Commands.parallel(
                    Commands.sequence(
                        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2))),
                        new DriveToPose(
                            drivetrain,
                            () -> Field2d.getInstance().getNearestBranch(Side.LEFT),
                            new Transform2d(
                                Units.inchesToMeters(2.0),
                                Units.inchesToMeters(0.5),
                                Rotation2d.fromDegrees(2.0))),
                        Commands.runOnce(
                            () -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3)))),
                    Commands.runOnce(elevator::goToSelectedPosition, elevator))
                .withName("drive to nearest left branch"));

    // drive to right branch of nearest reef face
    oi.getPrepToScoreCoralRightButton()
        .onTrue(
            Commands.parallel(
                    Commands.sequence(
                        Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 2))),
                        new DriveToPose(
                            drivetrain,
                            () -> Field2d.getInstance().getNearestBranch(Side.RIGHT),
                            new Transform2d(
                                Units.inchesToMeters(2.0),
                                Units.inchesToMeters(0.5),
                                Rotation2d.fromDegrees(2.0))),
                        Commands.runOnce(
                            () -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3)))),
                    Commands.runOnce(elevator::goToSelectedPosition, elevator))
                .withName("drive to nearest right branch"));
  }

  private static Command getScoreCoralCommand(Manipulator manipulator) {
    return Commands.sequence(
        Commands.either(
            Commands.runOnce(manipulator::scoreCoralThroughFunnel, manipulator),
            Commands.runOnce(manipulator::shootCoral, manipulator),
            () -> OISelector.getOperatorInterface().getLevel1Trigger().getAsBoolean()),
        Commands.waitUntil(() -> !manipulator.hasCoral()));
  }
}
