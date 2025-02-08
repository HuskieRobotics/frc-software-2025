package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO io;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Climber/TestingMode", 0);
  private final LoggedTunableNumber climberVoltage =
      new LoggedTunableNumber("Climber/Voltage", 0.0);

  public Climber(ClimberIO io) {
    this.io = io;
  }

  // getPosition() is placeholder
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    if (testingMode.get() == 1) {
      io.setVoltage(climberVoltage.get());
    } else if (inputs.positionInches > ClimberConstants.MAX_HEIGHT_INCHES) {
      stop();
    } else if (inputs.positionInches < ClimberConstants.MIN_HEIGHT_INCHES) {
      stop();
    }
  }

  public void extend() {
    io.setVoltage(ClimberConstants.EXTEND_VOLTAGE);
  }

  public void retract() {
    io.setVoltage(ClimberConstants.RETRACT_VOLTAGE);
  }

  public void retractSlow() {
    io.setVoltage(ClimberConstants.RETRACT_VOLTAGE_SLOW);
  }

  public void reset() {
    io.setVoltage(ClimberConstants.RESET_VOLTAGE);
  }

  public void stop() {
    io.setVoltage(0);
  }

  public void zero() {
    io.zeroPosition();
  }

  public double getPosition() {
    return inputs.positionInches;
  }
}
