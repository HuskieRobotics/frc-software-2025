package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.manipulator.Manipulator;

public class ClimberCommandFactory {

  private ClimberCommandFactory() {}

  public static void registerCommands(
      OperatorInterface oi, Climber climber, Manipulator manipulator) {

    // take in the manipulator subsystem here as well and call the openFunnelFlap method along with
    // cage catcher extension
    oi.getExtendCageCatcherButton()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(manipulator::disableFunnelForClimb, manipulator),
                    Commands.runOnce(climber::extendCageCatcher, climber),
                    Commands.waitUntil(climber::cageCatcherReleased),
                    Commands.runOnce(climber::stop, climber))
                .withName("extend cage catcher"));

    oi.getInitiateClimbButton()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(climber::climb, climber),
                    Commands.waitUntil(
                        () ->
                            (climber.climberLimitSwitchEngaged()
                                || climber.getPosition()
                                    > ClimberConstants.HARDSTOP_POSITION_INCHES)),
                    Commands.runOnce(climber::stop, climber))
                .withName("finish climb"));

    oi.getExtendClimberToMatchPositionButton()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(climber::zero, climber),
                    Commands.runOnce(climber::extendSlow, climber),
                    Commands.waitUntil(
                        () -> climber.getPosition() < -ClimberConstants.MAX_HEIGHT_INCHES),
                    Commands.runOnce(climber::stop, climber),
                    Commands.runOnce(climber::zero, climber))
                .withName("extend climber to match position"));

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
    oi.getExtendClimberSlowButton()
        .onFalse(Commands.runOnce(climber::stop).withName("extend climber slow stop climber"));

    oi.getZeroClimberButton()
        .onTrue(Commands.runOnce(climber::zero, climber).withName("zero climber"));
  }
}
