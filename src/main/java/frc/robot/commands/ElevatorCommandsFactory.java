package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommandsFactory {

  private ElevatorCommandsFactory() {}

  public static void registerCommands(OperatorInterface oi, Elevator elevator) {

    oi.getPrepareElevatorToScoreButton()
        .onTrue(
            Commands.runOnce(elevator::goToSelectedPosition, elevator)
                .withName("raise elevator to score"));
  }
}
