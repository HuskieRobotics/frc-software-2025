package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.manipulator.Manipulator;

public class CrossSubsystemsCommandsFactory {

  private static final LoggedTunableNumber removeAlgae =
      new LoggedTunableNumber("Manipulator/removeAlgae", 0);

  private static final LoggedTunableNumber scoreL1 =
      new LoggedTunableNumber("Manipulator/scoreL1", 0);

  private CrossSubsystemsCommandsFactory() {}

  public static void registerCommands(
      OperatorInterface oi, Elevator elevator, Manipulator manipulator) {
    oi.getScoreCoralButton()
        .onTrue(
            Commands.sequence(
                    Commands.either(
                        Commands.parallel(
                            Commands.runOnce(manipulator::removeAlgae),
                            Commands.sequence(
                                getScoreCoralCommand(manipulator),
                                Commands.runOnce(
                                    () ->
                                        elevator.goToPosition(
                                            ElevatorConstants.ReefBranch.BELOW_ALGAE_2),
                                    elevator),
                                Commands.waitUntil(
                                    () ->
                                        elevator.isAtPosition(
                                            ElevatorConstants.ReefBranch.BELOW_ALGAE_2)),
                                Commands.runOnce(
                                    () ->
                                        elevator.goToPosition(
                                            ElevatorConstants.ReefBranch.ABOVE_ALGAE_2),
                                    elevator), // FIXME: change to algae removal position
                                Commands.waitUntil(
                                    () ->
                                        elevator.isAtPosition(
                                            ElevatorConstants.ReefBranch.ABOVE_ALGAE_2)),
                                Commands.waitSeconds(2.0),
                                Commands.runOnce(manipulator::algaeIsRemoved))),
                        getScoreCoralCommand(manipulator),
                        () -> {
                          return removeAlgae.get() == 1;
                        }),
                    Commands.runOnce(
                        () -> elevator.goToPosition(ElevatorConstants.ReefBranch.HARDSTOP),
                        elevator))
                .withName("score coral"));
  }

  private static Command getScoreCoralCommand(Manipulator manipulator) {
    return Commands.sequence(
        Commands.either(
            Commands.runOnce(manipulator::scoreCoralThroughFunnel, manipulator),
            Commands.runOnce(manipulator::shootCoral, manipulator),
            () -> {
              return scoreL1.get() == 1;
            }),
        Commands.waitUntil(() -> !manipulator.hasCoral()));
  }
}
