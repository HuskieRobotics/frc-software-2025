package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ElevatorCommandsFactory {

  private ElevatorCommandsFactory() {}

  public static void registerCommands(OperatorInterface oi, Elevator elevator) {

    // consistent, extend button (hold)
    oi.getPrepareElevatorToScoreButton()
        .onTrue(
            Commands.runOnce(() -> elevator.goToPosition(ElevatorConstants.ReefBranch.L4), elevator)
                .withName("raise elevator to score"));
  }
}
