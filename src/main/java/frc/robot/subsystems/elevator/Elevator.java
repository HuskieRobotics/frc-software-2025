package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3061.drivetrain.DrivetrainConstants;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.operator_interface.OISelector;
import frc.robot.subsystems.elevator.ElevatorConstants.ReefBranch;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private ElevatorIO elevatorIO;
  private ReefBranch targetPosition = ReefBranch.HARDSTOP;

  // arbitrary high value
  private double distanceFromReef = 100.0;
  private InterpolatingDoubleTreeMap l2HeightMap;
  private InterpolatingDoubleTreeMap l3HeightMap;

  private Alert hardStopAlert =
      new Alert("Elevator position not 0 at bottom. Check belts for slipping.", AlertType.kError);

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
    this.l2HeightMap = new InterpolatingDoubleTreeMap();
    this.l3HeightMap = new InterpolatingDoubleTreeMap();

    populateL2Map();
    populateL3Map();

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
    Logger.recordOutput(SUBSYSTEM_NAME + "/distanceFromReef", distanceFromReef);

    current.calculate(Math.abs(inputs.statorCurrentAmpsLead));

    // FIXME: consider y to be on target as well
    if (Math.abs(distanceFromReef) < FAR_SCORING_DISTANCE
        && distanceFromReef > DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE) {
      LEDs.getInstance().requestState(LEDs.States.READY_TO_SCORE_FARTHER_AWAY);
    }

    if (testingMode.get() == 1) {

      if (elevatorVoltage.get() != 0) {
        elevatorIO.setMotorVoltage(elevatorVoltage.get());
      } else if (elevatorHeightInches.get() != 0) {
        elevatorIO.setPosition(Inches.of(elevatorHeightInches.get()));
      }
    }

    // Record cycle time
    LoggedTracer.record("Elevator");
  }

  private Distance reefBranchToDistance(ReefBranch reefBranch) {

    Distance height;

    switch (reefBranch) {
      case L1:
        height = L1_HEIGHT; // MIN_HEIGHT previously
        break;

      case ABOVE_L1:
        height = ABOVE_L1_HEIGHT;
        break;

      case L2:
        height = L2_HEIGHT;
        break;

      case MAX_L2:
        height = FAR_L2_HEIGHT;
        break;

      case L3:
        height = L3_HEIGHT;
        break;

      case MAX_L3:
        height = FAR_L3_HEIGHT;
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
        height = MIN_HEIGHT;
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

  public void populateL2Map() {
    l2HeightMap.put(FAR_SCORING_DISTANCE, FAR_L2_HEIGHT.in(Inches));
    l2HeightMap.put(0.0, L2_HEIGHT.in(Inches));
  }

  public void populateL3Map() {
    l3HeightMap.put(FAR_SCORING_DISTANCE, FAR_L2_HEIGHT.in(Inches));
    l3HeightMap.put(0.0, L3_HEIGHT.in(Inches));
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
    if (getSelectedPosition() == ReefBranch.L2 || getSelectedPosition() == ReefBranch.L3) {
      adjustPositionFromInterpolation();
    } else {
      goToPosition(getSelectedPosition());
    }
  }

  private void adjustPositionFromInterpolation() {
    // 4.5 inches is 1 coral away from the reef, we should probably be able to shoot up to 6 away
    if (getSelectedPosition() == ReefBranch.L2) {
      if (Math.abs(distanceFromReef) > FAR_SCORING_DISTANCE) {
        goToPosition(ReefBranch.MAX_L2);
      } else {
        elevatorIO.setPosition(Inches.of(l2HeightMap.get(Math.abs(distanceFromReef))));
      }
    } else if (getSelectedPosition() == ReefBranch.L3) {
      if (Math.abs(distanceFromReef) > FAR_SCORING_DISTANCE) {
        goToPosition(ReefBranch.MAX_L3);
      } else {
        elevatorIO.setPosition(Inches.of(l3HeightMap.get(Math.abs(distanceFromReef))));
      }
    }
  }

  public void setDistanceFromReef(double distance) {
    distanceFromReef = distance;
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

  public Command getElevatorLowerAndResetCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> goToPosition(ReefBranch.HARDSTOP)),
        Commands.waitUntil(
            () -> getPosition().in(Inches) < JUST_ABOVE_HARDSTOP.in(Inches) + TOLERANCE_INCHES),
        Commands.runOnce(() -> elevatorIO.setMotorVoltage(ELEVATOR_LOWERING_VOLTAGE)),
        Commands.waitUntil(
            () -> Math.abs(current.lastValue()) > STALL_CURRENT || Constants.getMode() == Mode.SIM),
        Commands.runOnce(() -> elevatorIO.setMotorVoltage(0)),
        Commands.runOnce(
            () -> hardStopAlert.set(Math.abs(getPosition().in(Inches)) > RESET_TOLERANCE)),
        Commands.runOnce(() -> elevatorIO.zeroPosition()));
  }
}
