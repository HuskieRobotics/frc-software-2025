package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.robot.subsystems.Climber.Climber;
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


  //ask about command syntax, what it means, explain example (if at all)
  
  public Command getPrepClimbSequence(Drivetrain drivetrain, Climber climber) {
    // require x subsystem to do __ (implement later)
    // ignore need for drivetrain for now
    // retract climber all the way (maybe)
    return Commands.runOnce(climber::reset, climber);
  }
  
  public Command getInitiateClimbButton(Drivetrain drivetrain, Climber climber) {
    //fix
    return Commands.runOnce(climber::placeholder, climber);
  }

  public Command zeroClimber(Drivetrain drivetrain, Climber climber) {
    //fix
    return Commands.runOnce(climber::zero, climber);
  }

  public Command raiseClimber(Drivetrain drivetrain, Climber climber) {
    //fix
    return Commands.runOnce(climber::extend, climber);
  }


}
