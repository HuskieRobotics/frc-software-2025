package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO io;

  private boolean extendingCageCatcher = false;
  private boolean retractingSlow = false;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Climber/TestingMode", 0);
  private final LoggedTunableNumber minHeight =
      new LoggedTunableNumber("Climber/MinHeight", ClimberConstants.MIN_HEIGHT_INCHES);
  private final LoggedTunableNumber climberVoltage =
      new LoggedTunableNumber("Climber/Voltage", 0.0);

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    if (testingMode.get() == 1) {
      io.setVoltage(climberVoltage.get());
    }

    // Record cycle time
    LoggedTracer.record("Climber");
  }

  public void extend() {
    this.extendingCageCatcher = false;
    io.setVoltage(ClimberConstants.EXTEND_VOLTAGE);
  }

  public void extendCageCatcher() {
    this.extendingCageCatcher = true;
    io.setVoltage(ClimberConstants.EXTEND_VOLTAGE);
  }

  public void retract() {
    io.setVoltage(ClimberConstants.RETRACT_VOLTAGE);
  }

  public void retractSlow() {
    retractingSlow = true;
    io.setVoltage(ClimberConstants.RETRACT_VOLTAGE_SLOW);
  }

  public void reset() {
    io.setVoltage(ClimberConstants.RESET_VOLTAGE);
  }

  public void stop() {
    retractingSlow = false;
    io.setVoltage(0);
  }

  public void zero() {
    io.zeroPosition();
  }

  // should we add a tolerance here for if the position slips a little bit?
  public boolean cageCatcherReleased() {
    return !this.extendingCageCatcher
        && inputs.positionInches > ClimberConstants.CAGE_CATCHER_EXTEND_POS_INCHES - 0.0;
  }

  public double getPosition() {
    return inputs.positionInches;
  }
}
