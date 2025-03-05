package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.operator_interface.OISelector;
import frc.robot.subsystems.elevator.ElevatorConstants.ReefBranch;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private ElevatorIO elevatorIO;
  private ReefBranch targetPosition = ReefBranch.HARDSTOP;

  private Alert hardStopAlert = new Alert("Elevator is not at zero position", AlertType.kError);

  private LinearFilter current =
      LinearFilter.singlePoleIIR(
          0.1, 0.02); // the first value is the time constant, the characteristic timescale of the
  // filter's impulse response, and the second value is the time-period, how often
  // the calculate() method will be called

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

    Logger.recordOutput(SUBSYSTEM_NAME + "/targetPosition", targetPosition);

    current.calculate(Math.abs(inputs.statorCurrentAmpsLead));

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
        height = MIN_HEIGHT;
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

      case ABOVE_ALGAE_1:
        height = ABOVE_ALGAE_1_HEIGHT;
        break;

      case BELOW_ALGAE_1:
        height = BELOW_ALGAE_1_HEIGHT;
        break;

      case ABOVE_ALGAE_2:
        height = ABOVE_ALGAE_2_HEIGHT;
        break;

      case BELOW_ALGAE_2:
        height = BELOW_ALGAE_2_HEIGHT;
        break;

      case HARDSTOP:
        height = JUST_ABOVE_HARDSTOP;
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
    targetPosition = reefBranch;
    elevatorIO.setPosition(reefBranchToDistance(reefBranch));
  }

  public Distance getPosition() {
    return Inches.of(inputs.positionInches);
  }

  private ReefBranch getSelectedPosition() {
    if (OISelector.getOperatorInterface().getLevel4Trigger().getAsBoolean()) {
      return ReefBranch.L4;
    } else if (OISelector.getOperatorInterface().getLevel3Trigger().getAsBoolean()) {
      return ReefBranch.L3;
    } else if (OISelector.getOperatorInterface().getLevel2Trigger().getAsBoolean()) {
      return ReefBranch.L2;
    } else {
      return ReefBranch.L1;
    }
  }

  public void goToSelectedPosition() {
    goToPosition(getSelectedPosition());
  }

  public boolean isAtSelectedPosition() {
    return isAtPosition(getSelectedPosition());
  }

  private ReefBranch getSelectedAlgaePosition() {
    if (OISelector.getOperatorInterface().getRemoveLowAlgaeTrigger().getAsBoolean()) {
      return ReefBranch.ALGAE_1;
    } else if (OISelector.getOperatorInterface().getRemoveHighAlgaeTrigger().getAsBoolean()) {
      return ReefBranch.ALGAE_2;
    } else {
      return ReefBranch.HARDSTOP;
    }
  }

  public boolean isAlgaePositionSelected() {
    return getSelectedAlgaePosition() != ReefBranch.HARDSTOP;
  }

  public void goBelowSelectedAlgaePosition() {
    if (getSelectedAlgaePosition() == ReefBranch.ALGAE_1) {
      goToPosition(ReefBranch.BELOW_ALGAE_1);
    } else if (getSelectedAlgaePosition() == ReefBranch.ALGAE_2) {
      goToPosition(ReefBranch.BELOW_ALGAE_2);
    }
  }

  public void goAboveSelectedAlgaePosition() {
    if (getSelectedAlgaePosition() == ReefBranch.ALGAE_1) {
      goToPosition(ReefBranch.ABOVE_ALGAE_1);
    } else if (getSelectedAlgaePosition() == ReefBranch.ALGAE_2) {
      goToPosition(ReefBranch.ABOVE_ALGAE_2);
    }
  }

  public boolean isBelowSelectedAlgaePosition() {
    if (getSelectedAlgaePosition() == ReefBranch.ALGAE_1) {
      return isAtPosition(ReefBranch.BELOW_ALGAE_1);
    } else if (getSelectedAlgaePosition() == ReefBranch.ALGAE_2) {
      return isAtPosition(ReefBranch.BELOW_ALGAE_2);
    } else {
      return true;
    }
  }

  public boolean isAboveSelectedAlgaePosition() {
    if (getSelectedAlgaePosition() == ReefBranch.ALGAE_1) {
      return isAtPosition(ReefBranch.ABOVE_ALGAE_1);
    } else if (getSelectedAlgaePosition() == ReefBranch.ALGAE_2) {
      return isAtPosition(ReefBranch.ABOVE_ALGAE_2);
    } else {
      return true;
    }
  }

  public void raiseElevatorSlow() {
    elevatorIO.setMotorVoltage(ELEVATOR_RAISE_SLOW_VOLTAGE);
  }

  public void lowerElevatorSlow() {
    elevatorIO.setMotorVoltage(ELEVATOR_LOWERING_SLOW_VOLTAGE);
  }

  public void stop() {
    elevatorIO.setMotorVoltage(0);
  }

  public void zero() {
    elevatorIO.zeroPosition();
  }

  public void checkHeight(){
    if (getPosition().in(Inches) > RESET_TOLERANCE){
      hardStopAlert.set(true);
    } 
    else {
      hardStopAlert.set(false);
    }
  }

  public Command getElevatorLowerAndResetCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> goToPosition(ReefBranch.HARDSTOP)),
        Commands.waitUntil(
            () -> getPosition().in(Inches) < JUST_ABOVE_HARDSTOP.in(Inches) + TOLERANCE_INCHES),
        Commands.runOnce(() -> elevatorIO.setMotorVoltage(ELEVATOR_LOWERING_VOLTAGE)),
        Commands.waitUntil(
            () -> Math.abs(current.lastValue()) > STALL_CURRENT || Constants.getMode() == Mode.SIM),
        Commands.runOnce(() -> elevatorIO.setMotorVoltage(0)),
        Commands.runOnce(this::checkHeight),
        Commands.runOnce(() -> elevatorIO.zeroPosition()));
  }
}
