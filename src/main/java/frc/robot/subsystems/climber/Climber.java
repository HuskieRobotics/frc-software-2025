package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO io;
  private static final String SUBSYSTEM_NAME = "Climber";

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
    Logger.processInputs(SUBSYSTEM_NAME, inputs);
    if (testingMode.get() == 1) {
      io.setVoltage(climberVoltage.get());
    } else if (inputs.positionInches > ClimberConstants.MAX_HEIGHT_INCHES) {
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

  // work lost 2/19/25, this is not accurate
  public Command getSystemCheckCommand() {
    return Commands.sequence(Commands.runOnce(() -> {}), getVelocityCheckCommand())
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .withName(SUBSYSTEM_NAME + "SystemCheck");
  }

  public Command getVelocityCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> io.setVoltage(climberVoltage.get())),
            Commands.waitSeconds(2),
            Commands.runOnce(
                () -> {
                  double actualVelocity = inputs.velocityInchesPerSecond;
                  // check conversion of velocity 2/19/25
                  double expectedVelocity = climberVoltage.get() * ClimberConstants.KVEXP;
                  if (Math.abs(actualVelocity - expectedVelocity)
                      > ClimberConstants.VELOCITY_TOLERANCE) {
                    FaultReporter.getInstance()
                        .addFault(SUBSYSTEM_NAME, "Velocity out of tolerance");
                  }
                }))
        .withName(SUBSYSTEM_NAME + "VelocityCheck");
  }
}
