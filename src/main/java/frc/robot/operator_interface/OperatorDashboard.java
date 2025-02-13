package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team6328.util.LoggedTunableBoolean;

public class OperatorDashboard implements OperatorInterface {

  public final LoggedTunableBoolean enablePrimaryIRSensors =
      new LoggedTunableBoolean("operatorDashboard/EnablePrimaryIRSensors", true, true);

  public final LoggedTunableBoolean level1 =
      new LoggedTunableBoolean("operatorDashboard/Level 1", false, true);
  public final LoggedTunableBoolean level2 =
      new LoggedTunableBoolean("operatorDashboard/Level 2", false, true);
  public final LoggedTunableBoolean level3 =
      new LoggedTunableBoolean("operatorDashboard/Level 3 ", false, true);
  public final LoggedTunableBoolean level4 =
      new LoggedTunableBoolean("operatorDashboard/Level 4 ", true, true);

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
                }));

    getLevel2Trigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  level1.set(false);
                  level3.set(false);
                  level4.set(false);
                }));

    getLevel3Trigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  level1.set(false);
                  level2.set(false);
                  level4.set(false);
                }));

    getLevel4Trigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  level1.set(false);
                  level2.set(false);
                  level3.set(false);
                }));
  }

  @Override
  public Trigger getEnablePrimaryIRSensors() {
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
}
