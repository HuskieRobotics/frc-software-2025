package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.subsystem.Subsystem;

public class ClimberCommandFactory {

  private ClimberCommandFactory() {}

  public static void registerCommands(OperatorInterface oi, Climber climber) {
    oi.getPrepClimbSequence()
        .onTrue(Commands.runOnce(climber::extend, climber).withName("extend climber"));
    oi.getInitiateClimbButton()
        .onTrue(Commands.runOnce(climber::retract, climber).withName("retract climber"));
    oi.getRetractClimberSlowButton()
        .onTrue(Commands.runOnce(climber::retractSlow, climber).withName("retract climber slow"));
    oi.getRetractClimberSlowButton()
        .onFalse(Commands.runOnce(climber::stop, climber).withName("stop climber"));
    oi.getZeroClimberButton()
        .onTrue(Commands.runOnce(climber::zero, climber).withName("zero climber"));
  }
  /**
   * Example method that creates an inline command for an example subsystem
   *
   * @param Subsystem subsystem
   * @return Command (example inline runOnce command)
   */
  public Command getExampleSubsystemCommand(Subsystem subsystem) {
    return Commands.runOnce(subsystem::exampleMethod, subsystem);
  }
}
