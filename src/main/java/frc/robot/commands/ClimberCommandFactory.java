package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.climber.Climber;

public class ClimberCommandFactory {

  private ClimberCommandFactory() {}

  // 2/8/2025 NOTE:
  // in AdvantageScope, position is constantly slowly decreasing
  // should be addressed eventually
  public static void registerCommands(OperatorInterface oi, Climber climber) {

    oi.getExtendCageCatcherButton()
        .onTrue(
            Commands.runOnce(climber::extendCageCatcher, climber).withName("extend cage catcher"));
    
    oi.getExtendClimberButton()
        .onTrue(
            Commands.either(
                Commands.runOnce(climber::extend, climber),
                Commands.none(),
                climber::cageCatcherReleased).withName("extend climber"));

    // inconsistent, retract button (single press) works after button spam / sometimes perfect
    // note: works consistently w/ a double/triple click, unsure why
    oi.getInitiateClimbButton()
        .onTrue(Commands.runOnce(climber::retract, climber).withName("retract climber"));

    // inconsistent, slow retract (hold) works w/ initial button spam and then holding for use
    oi.getRetractClimberSlowButton()
        .onTrue(Commands.runOnce(climber::retractSlow, climber).withName("retract climber slow"));
    oi.getRetractClimberSlowButton()
        .onFalse(Commands.sequence(Commands.runOnce(climber::stop, climber), Commands.runOnce(climber::zero, climber)).withName("stop and zero climber"));

    // consistent, zero button (single press)
    oi.getZeroClimberButton()
        .onTrue(Commands.runOnce(climber::zero, climber).withName("zero climber"));
  }
}
