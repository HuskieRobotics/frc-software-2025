package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.util.SysIdRoutineChooser;
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

  public Elevator(ElevatorIO io) {

    this.elevatorIO = io;

    io.zeroPosition();

    // Add sysId routine for each stage of the elevator

    SysIdRoutineChooser.getInstance().addOption("Subsystem Voltage", sysIdRoutineStage1);

    SysIdRoutineChooser.getInstance().addOption("Subsystem Voltage", sysIdRoutineStage2);

    SysIdRoutineChooser.getInstance().addOption("Subsystem Voltage", sysIdRoutineStage3);

    FaultReporter.getInstance()
        .registerSystemCheck(SUBSYSTEM_NAME, getElevatorSystemCheckCommand());
  }

  private final SysIdRoutine sysIdRoutineStage1 =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              null, // Use default step voltage (7 V)
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> elevatorIO.setMotorVoltage(output.in(Volts)), null, this));

  private final SysIdRoutine sysIdRoutineStage2 =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              null, // Use default step voltage (7 V)
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> elevatorIO.setMotorVoltage(output.in(Volts)), null, this));

  private final SysIdRoutine sysIdRoutineStage3 =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              null, // Use default step voltage (7 V)
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

      case L3:
        height = L3_HEIGHT;
        break;

      case L4:
        height = L4_HEIGHT;
        break;

      case ALGAE_1:
        height = ALGAE1_HEIGHT;
        break;

      case ALGAE_2:
        height = ALGAE2_HEIGHT;
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

  public Distance getPosition() {

    return Inches.of(inputs.positionInches);
  }
}
