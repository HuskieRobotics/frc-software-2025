package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.subsystem.Subsystem;

public class SubsystemCommandFactory {

  private SubsystemCommandFactory() {}

  /**
   * Example method that creates an inline command for an example subsystem
   *
   * @param Subsystem subsystem
   * @return Command (example inline runOnce command)
   */
  public Command getExampleSubsystemCommand(Subsystem subsystem) {
    return Commands.runOnce(subsystem::exampleMethod, subsystem);
  }

  public Command getIntakeDeployCommand(Subsystem Intake) {
    return Commands.runOnce(Intake::deploy, Intake);
  }

  public Command getIntakeScoreCommand(Subsystem Intake) {
    return Commands.runOnce(Intake::shootOut, Intake);
  }
}
