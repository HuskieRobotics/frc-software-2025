package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;

public class ClimberCommandFactory {

  private static final LoggedTunableNumber minHeight =
      new LoggedTunableNumber("Climber/MinHeight", ClimberConstants.MIN_HEIGHT_INCHES);

  private ClimberCommandFactory() {}

  public static void registerCommands(OperatorInterface oi, Climber climber) {

    // take in the manipulator subsystem here as well and call the openFunnelFlap method along with
    // cage catcher extension
    oi.getExtendCageCatcherButton()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(climber::extendCageCatcher, climber),
                    Commands.waitUntil(climber::cageCatcherReleased),
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

    oi.getExtendClimberSlowButton()
        .onTrue(Commands.runOnce(climber::extendSlow, climber).withName("extend climber slow"));
    oi.getExtendClimberSlowButton().onFalse(Commands.runOnce(climber::stop));
  }
}
