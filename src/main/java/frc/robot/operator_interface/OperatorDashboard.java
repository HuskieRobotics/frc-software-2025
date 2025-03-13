package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team6328.util.LoggedTunableBoolean;

public class OperatorDashboard implements OperatorInterface {
  public final LoggedTunableBoolean enableVision =
      new LoggedTunableBoolean("operatorDashboard/Enable Vision", true, true);

  public final LoggedTunableBoolean enablePrimaryIRSensors =
      new LoggedTunableBoolean("operatorDashboard/Enable Primary IR Sensors", true, true);

  public final LoggedTunableBoolean level1 =
      new LoggedTunableBoolean("operatorDashboard/Level 1", false, true);
  public final LoggedTunableBoolean level2 =
      new LoggedTunableBoolean("operatorDashboard/Level 2", false, true);
  public final LoggedTunableBoolean level3 =
      new LoggedTunableBoolean("operatorDashboard/Level 3 ", false, true);
  public final LoggedTunableBoolean level4 =
      new LoggedTunableBoolean("operatorDashboard/Level 4 ", true, true);

  private final LoggedTunableBoolean algaeBarge =
      new LoggedTunableBoolean("operatorDashboard/Barge", false, true);
  private final LoggedTunableBoolean algaeProcessor =
      new LoggedTunableBoolean("operatorDashboard/Processor", false, true);
  private final LoggedTunableBoolean algaeDrop =
      new LoggedTunableBoolean("operatorDashboard/Drop", false, true);

  private final LoggedTunableBoolean reefBranchA =
      new LoggedTunableBoolean("operatorDashboard/A", false, true);
  private final LoggedTunableBoolean reefBranchB =
      new LoggedTunableBoolean("operatorDashboard/B", false, true);
  private final LoggedTunableBoolean reefBranchC =
      new LoggedTunableBoolean("operatorDashboard/C", false, true);
  private final LoggedTunableBoolean reefBranchD =
      new LoggedTunableBoolean("operatorDashboard/D", false, true);
  private final LoggedTunableBoolean reefBranchE =
      new LoggedTunableBoolean("operatorDashboard/E", false, true);
  private final LoggedTunableBoolean reefBranchF =
      new LoggedTunableBoolean("operatorDashboard/F", false, true);
  private final LoggedTunableBoolean reefBranchG =
      new LoggedTunableBoolean("operatorDashboard/G", false, true);
  private final LoggedTunableBoolean reefBranchH =
      new LoggedTunableBoolean("operatorDashboard/H", false, true);
  private final LoggedTunableBoolean reefBranchI =
      new LoggedTunableBoolean("operatorDashboard/I", false, true);
  private final LoggedTunableBoolean reefBranchJ =
      new LoggedTunableBoolean("operatorDashboard/J", false, true);
  private final LoggedTunableBoolean reefBranchK =
      new LoggedTunableBoolean("operatorDashboard/K", false, true);
  private final LoggedTunableBoolean reefBranchL =
      new LoggedTunableBoolean("operatorDashboard/L", false, true);

  private final LoggedTunableBoolean[] reefBranches = {
    reefBranchA,
    reefBranchB,
    reefBranchC,
    reefBranchD,
    reefBranchE,
    reefBranchF,
    reefBranchG,
    reefBranchH,
    reefBranchI,
    reefBranchJ,
    reefBranchK,
    reefBranchL
  };

  public OperatorDashboard() {

    // The controls to select reef branch levels must be mutually exclusive; if one is selected
    // (true), the others must be set to false.
    getLevel1Trigger()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      level2.set(false);
                      level3.set(false);
                      level4.set(false);
                    })
                .ignoringDisable(true));

    getLevel2Trigger()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      level1.set(false);
                      level3.set(false);
                      level4.set(false);
                    })
                .ignoringDisable(true));

    getLevel3Trigger()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      level1.set(false);
                      level2.set(false);
                      level4.set(false);
                    })
                .ignoringDisable(true));

    getLevel4Trigger()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      level1.set(false);
                      level2.set(false);
                      level3.set(false);
                    })
                .ignoringDisable(true));

    getAlgaeBargeTrigger()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          algaeProcessor.set(false);
                          algaeDrop.set(false);
                        }))
                .ignoringDisable(true));

    getAlgaeDropTrigger()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          algaeProcessor.set(false);
                          algaeBarge.set(false);
                        }))
                .ignoringDisable(true));

    getAlgaeProcessorTrigger()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          algaeBarge.set(false);
                          algaeDrop.set(false);
                        }))
                .ignoringDisable(true));

    getReefBranchATrigger().onTrue(getToggleReefBranch(reefBranchA));
    getReefBranchBTrigger().onTrue(getToggleReefBranch(reefBranchB));
    getReefBranchCTrigger().onTrue(getToggleReefBranch(reefBranchC));
    getReefBranchDTrigger().onTrue(getToggleReefBranch(reefBranchD));
    getReefBranchETrigger().onTrue(getToggleReefBranch(reefBranchE));
    getReefBranchFTrigger().onTrue(getToggleReefBranch(reefBranchF));
    getReefBranchGTrigger().onTrue(getToggleReefBranch(reefBranchG));
    getReefBranchHTrigger().onTrue(getToggleReefBranch(reefBranchH));
    getReefBranchITrigger().onTrue(getToggleReefBranch(reefBranchI));
    getReefBranchJTrigger().onTrue(getToggleReefBranch(reefBranchJ));
    getReefBranchKTrigger().onTrue(getToggleReefBranch(reefBranchK));
    getReefBranchLTrigger().onTrue(getToggleReefBranch(reefBranchL));
  }

  @Override
  public Trigger getVisionIsEnabledTrigger() {
    return new Trigger(() -> enableVision.get());
  }

  @Override
  public Trigger getEnablePrimaryIRSensorsTrigger() {
    return new Trigger(() -> enablePrimaryIRSensors.get());
  }

  @Override
  public Trigger getLevel1Trigger() {
    return new Trigger(() -> level1.get());
  }

  @Override
  public Trigger getLevel2Trigger() {
    return new Trigger(() -> level2.get());
  }

  @Override
  public Trigger getLevel3Trigger() {
    return new Trigger(() -> level3.get());
  }

  @Override
  public Trigger getLevel4Trigger() {
    return new Trigger(() -> level4.get());
  }

  @Override
  public Trigger getAlgaeBargeTrigger() {
    return new Trigger(() -> algaeBarge.get());
  }

  @Override
  public Trigger getAlgaeProcessorTrigger() {
    return new Trigger(() -> algaeProcessor.get());
  }

  @Override
  public Trigger getAlgaeDropTrigger() {
    return new Trigger(() -> algaeDrop.get());
  }

  @Override
  public Trigger getReefBranchATrigger() {
    return new Trigger(() -> reefBranchA.get());
  }

  @Override
  public Trigger getReefBranchBTrigger() {
    return new Trigger(() -> reefBranchB.get());
  }

  @Override
  public Trigger getReefBranchCTrigger() {
    return new Trigger(() -> reefBranchC.get());
  }

  @Override
  public Trigger getReefBranchDTrigger() {
    return new Trigger(() -> reefBranchD.get());
  }

  @Override
  public Trigger getReefBranchETrigger() {
    return new Trigger(() -> reefBranchE.get());
  }

  @Override
  public Trigger getReefBranchFTrigger() {
    return new Trigger(() -> reefBranchF.get());
  }

  @Override
  public Trigger getReefBranchGTrigger() {
    return new Trigger(() -> reefBranchG.get());
  }

  @Override
  public Trigger getReefBranchHTrigger() {
    return new Trigger(() -> reefBranchH.get());
  }

  @Override
  public Trigger getReefBranchITrigger() {
    return new Trigger(() -> reefBranchI.get());
  }

  @Override
  public Trigger getReefBranchJTrigger() {
    return new Trigger(() -> reefBranchJ.get());
  }

  @Override
  public Trigger getReefBranchKTrigger() {
    return new Trigger(() -> reefBranchK.get());
  }

  @Override
  public Trigger getReefBranchLTrigger() {
    return new Trigger(() -> reefBranchL.get());
  }

  private Command getToggleReefBranch(LoggedTunableBoolean selectedReefBranch) {
    return Commands.runOnce(
            () -> {
              for (LoggedTunableBoolean branch : reefBranches) {
                if (branch != selectedReefBranch) {
                  branch.set(false);
                }
              }
            })
        .ignoringDisable(true);
  }
}
