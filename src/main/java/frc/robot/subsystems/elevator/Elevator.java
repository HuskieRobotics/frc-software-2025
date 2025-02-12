package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTunableBoolean;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.subsystems.elevator.ElevatorConstants.ReefBranch;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private ElevatorIO elevatorIO;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final LoggedTunableNumber testingMode =
      new LoggedTunableNumber("Elevator/TestingMode", 0);

  private final LoggedTunableNumber elevatorVoltage =
      new LoggedTunableNumber("Elevator/Voltage", 0);

  private final LoggedTunableNumber elevatorHeightInches =
      new LoggedTunableNumber("Elevator/Height(Inches)", 0);

  private final LoggedTunableBoolean l1Command =
      new LoggedTunableBoolean("Elevator/L1Command", false, true);
  private final LoggedTunableBoolean l2Command =
      new LoggedTunableBoolean("Elevator/L2Command", false, true);
  private final LoggedTunableBoolean l3Command =
      new LoggedTunableBoolean("Elevator/L3Command", false, true);
  private final LoggedTunableBoolean l4Command =
      new LoggedTunableBoolean("Elevator/L4Command", false, true);

  private final LoggedTunableBoolean algae1LowCommand =
      new LoggedTunableBoolean("Elevator/Algae1LowCommand", false, true);
  private final LoggedTunableBoolean algae1HighCommand =
      new LoggedTunableBoolean("Elevator/Algae1HighCommand", false, true);
  private final LoggedTunableBoolean algae2LowCommand =
      new LoggedTunableBoolean("Elevator/Algae2LowCommand", false, true);
  private final LoggedTunableBoolean algae2HighCommand =
      new LoggedTunableBoolean("Elevator/Algae2HighCommand", false, true);

  public Elevator(ElevatorIO io) {

    this.elevatorIO = io;

    io.zeroPosition();

    // Add sysId routine for each stage of the elevator

    SysIdRoutineChooser.getInstance().addOption("Elevator Voltage 1", sysIdRoutineStage1);

    SysIdRoutineChooser.getInstance().addOption("Elevator Voltage 2", sysIdRoutineStage2);

    SysIdRoutineChooser.getInstance().addOption("Elevator Voltage 3", sysIdRoutineStage3);

    // FaultReporter.getInstance()
    //     .registerSystemCheck(SUBSYSTEM_NAME, getElevatorSystemCheckCommand());
  }

  private final SysIdRoutine sysIdRoutineStage1 =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(2.0).per(Second), // Use default ramp rate (1 V/s)
              Volts.of(2.0), // Use default step voltage (7 V)
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> elevatorIO.setMotorVoltage(output.in(Volts)), null, this));

  private final SysIdRoutine sysIdRoutineStage2 =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(2).per(Second), // Use default ramp rate (1 V/s)
              Volts.of(2), // Use default step voltage (7 V)
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> elevatorIO.setMotorVoltage(output.in(Volts)), null, this));

  private final SysIdRoutine sysIdRoutineStage3 =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(2).per(Second), // Use default ramp rate (1 V/s)
              Volts.of(2), // Use default step voltage (7 V)
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> elevatorIO.setMotorVoltage(output.in(Volts)), null, this));

  @Override
  public void periodic() {
    elevatorIO.updateInputs(inputs);
    Logger.processInputs(SUBSYSTEM_NAME, inputs);

    if (testingMode.get() == 1) {

      if (elevatorVoltage.get() != 0) {
        elevatorIO.setMotorVoltage(elevatorVoltage.get());
      } else if (elevatorHeightInches.get() != 0) {
        elevatorIO.setPosition(Inches.of(elevatorHeightInches.get()));
      }
    }
  }

  private Distance reefBranchToDistance(ReefBranch reefBranch) {

    Distance height;

    switch (reefBranch) {
      case L1:
        height = L1_HEIGHT;
        break;

      case L2:
        height = L2_HEIGHT;
        break;

      case L2_1_CORAL_AWAY:
        height = L2_HEIGHT_1_CORAL_AWAY;
        break;

      case L3:
        height = L3_HEIGHT;
        break;

      case L3_1_CORAL_AWAY:
        height = L3_HEIGHT_1_CORAL_AWAY;
        break;

      case L4:
        height = L4_HEIGHT;
        break;

      case ALGAE_1_LOW:
        height = ALGAE1_HEIGHT_LOW;
        break;

      case ALGAE_1_HIGH:
        height = ALGAE1_HEIGHT_HIGH;
        break;

      case ALGAE_2_LOW:
        height = ALGAE2_HEIGHT_LOW;
        break;

      case ALGAE_2_HIGH:
        height = ALGAE2_HEIGHT_HIGH;
        break;

      default:
        height = MIN_HEIGHT;
        break;
    }
    return height;
  }

  public boolean isAtPosition(ReefBranch reefBranch) {

    return getPosition().minus(reefBranchToDistance(reefBranch)).abs(Inches) < TOLERANCE_INCHES;
  }

  // TODO: Implement system check method
  public Command getElevatorSystemCheckCommand() {
    return null;
  }

  public void goToPosition(ReefBranch reefBranch) {
    elevatorIO.setPosition(reefBranchToDistance(reefBranch));
  }

  public void goToPositionWithCommand() {
    if (l1Command.get()) {
      goToPosition(ReefBranch.L1);
    } else if (l2Command.get()) {
      goToPosition(ReefBranch.L2);
    } else if (l3Command.get()) {
      goToPosition(ReefBranch.L3);
    } else if (l4Command.get()) {
      goToPosition(ReefBranch.L4);
    } else if (algae1LowCommand.get()) {
      goToPosition(ReefBranch.ALGAE_1_LOW);
    } else if (algae1HighCommand.get()) {
      goToPosition(ReefBranch.ALGAE_1_HIGH);
    } else if (algae2LowCommand.get()) {
      goToPosition(ReefBranch.ALGAE_2_LOW);
    } else if (algae2HighCommand.get()) {
      goToPosition(ReefBranch.ALGAE_2_HIGH);
    }
  }

  public Distance getPosition() {
    return Inches.of(inputs.positionInches);
  }
}
