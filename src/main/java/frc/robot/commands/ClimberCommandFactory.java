package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;

public class ClimberCommandFactory {

  private ClimberCommandFactory() {}

  // 2/8/2025 NOTE:
  // in AdvantageScope, position is constantly slowly decreasing
  // should be addressed eventually
  public static void registerCommands(OperatorInterface oi, Climber climber) {

    oi.getExtendCageCatcherButton()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(climber::extendCageCatcher, climber),
                    Commands.waitUntil(
                        () ->
                            climber.getPosition()
                                > ClimberConstants.CAGE_CATCHER_EXTEND_POS_INCHES),
                    Commands.runOnce(climber::stopExtension, climber))
                .withName("extend cage catcher"));

    oi.getInitiateClimbButton()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(climber::climb, climber),
                    Commands.waitUntil(
                        () -> climber.getPosition() > ClimberConstants.MAX_HEIGHT_INCHES),
                    Commands.runOnce(climber::stop, climber))
                .withName("retract climber"));

    oi.getRetractClimberSlowButton()
        .onTrue(Commands.runOnce(climber::retractSlow, climber).withName("retract climber slow"));
    oi.getRetractClimberSlowButton()
        .onFalse(
            Commands.sequence(
                    Commands.runOnce(climber::stop, climber),
                    Commands.runOnce(climber::zero, climber))
                .withName("stop and zero climber"));

    // FIXME: didn't we get rid of this button???
    // consistent, zero button (single press)
    oi.getZeroClimberButton()
        .onTrue(Commands.runOnce(climber::zero, climber).withName("zero climber"));
  }
}
