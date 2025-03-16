package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO io;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Climber/TestingMode", 0);
  private final LoggedTunableNumber minHeight =
      new LoggedTunableNumber("Climber/MinHeight", ClimberConstants.MIN_HEIGHT_INCHES);
  private final LoggedTunableNumber maxHeight =
      new LoggedTunableNumber("Climber/MaxHeight", ClimberConstants.MAX_HEIGHT_INCHES);
  private final LoggedTunableNumber cageCatcherExtendPos =
      new LoggedTunableNumber(
          "Climber/CageCatcherExtendPos", ClimberConstants.CAGE_CATCHER_EXTEND_POS_INCHES);

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

  public void extendCageCatcher() {
    io.unlockServo();
    io.setVoltage(ClimberConstants.CLIMB_VOLTAGE);
  }

  public void climb() {
    io.unlockServo();
    io.setVoltage(ClimberConstants.CLIMB_VOLTAGE);
  }

  public void retractSlow() {
    io.unlockServo();
    io.setVoltage(ClimberConstants.RETRACT_VOLTAGE_SLOW);
  }

  public void extendSlow() {
    io.unlockServo();
    io.setVoltage(ClimberConstants.EXTEND_VOLTAGE);
  }

  public void stop() {
    io.lockServo();
    io.setVoltage(0);
  }

  public void stopExtension() {
    io.setVoltage(0);
  }

  public void zero() {
    io.zeroPosition();
  }

  // should we add a tolerance here for if the position slips a little bit?
  public boolean cageCatcherReleased() {
    return inputs.positionInches > (ClimberConstants.CAGE_CATCHER_EXTEND_POS_INCHES - 0.5);
  }

  public double getPosition() {
    return inputs.positionInches;
  }
}
