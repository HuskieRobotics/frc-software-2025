package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ElevatorCommandsFactory {

  private ElevatorCommandsFactory() {}

  public static void registerCommands(OperatorInterface oi, Elevator elevator) {

    oi.getPrepareElevatorToScoreButton()
        .onTrue(
            Commands.runOnce(elevator::goToSelectedPosition, elevator)
                .withName("raise elevator to score"));

    oi.raiseElevatorSlow()
        .onTrue(
            Commands.runOnce(elevator::raiseElevatorSlow, elevator)
                .withName("raise elevator slow"));
  oi.lowerElevatorSlow()
        .onTrue(
            Commands.runOnce(elevator::lowerElevatorSlow, elevator)
                .withName("lower elevator slow"));
  oi.lowerElevatorSlow()
        .onFalse(
            Commands.sequence(
                Commands.runOnce(elevator::stop, elevator),
                Commands.runOnce(elevator::zero, elevator)
                    .withName("stop and zero elevator")
            )
                .withName("stop and zero elevator"));
  }
}

