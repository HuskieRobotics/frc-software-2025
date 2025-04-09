package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import frc.robot.Field2d;
import frc.robot.Field2d.AlgaePosition;
import frc.robot.operator_interface.OISelector;
import frc.robot.subsystems.elevator.ElevatorConstants.ScoringHeight;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private ElevatorIO elevatorIO;
  private ScoringHeight targetPosition = ScoringHeight.HARDSTOP;

  // instantiate distanceFromReef with arbitrary far values
  private Transform2d distanceFromReef = new Transform2d(100.0, 100.0, Rotation2d.fromDegrees(180));

  private Alert hardStopAlert =
      new Alert("Elevator position not 0 at bottom. Check belts for slipping.", AlertType.kError);

  private Alert jammedAlert =
      new Alert("Elevator jam detected. Use manual control.", AlertType.kError);

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
    Logger.recordOutput(SUBSYSTEM_NAME + "/distanceFromReef", distanceFromReef);

    if (current.calculate(Math.abs(inputs.statorCurrentAmpsLead)) > JAMMED_CURRENT) {
      CommandScheduler.getInstance()
          .schedule(
              Commands.sequence(
                  Commands.runOnce(() -> elevatorIO.setMotorVoltage(0)),
                  Commands.run(() -> LEDs.getInstance().requestState(LEDs.States.ELEVATOR_JAMMED))
                      .withTimeout(1.0)));
      jammedAlert.set(true);
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

  private Distance reefBranchToDistance(ScoringHeight reefBranch) {

    Distance height;

    switch (reefBranch) {
      case L1:
        height = L1_HEIGHT;
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

      case LOW_ALGAE:
        height = LOW_ALGAE_HEIGHT;
        break;

      case HIGH_ALGAE:
        height = HIGH_ALGAE_HEIGHT;
        break;

      case BELOW_HIGH_ALGAE:
        height = BELOW_HIGH_ALGAE_HEIGHT;
        break;

      case BELOW_LOW_ALGAE:
        height = BELOW_LOW_ALGAE_HEIGHT;
        break;

      case HARDSTOP:
        height = MIN_HEIGHT;
        break;

      case BARGE:
        height = BARGE_HEIGHT;
        break;

      case PROCESSOR:
        height = PROCESSOR_HEIGHT;
        break;

      default:
        height = MIN_HEIGHT;
        break;
    }
    return height;
  }

  public boolean isAtPosition(ScoringHeight reefBranch) {

    return getPosition().minus(reefBranchToDistance(reefBranch)).abs(Inches) < TOLERANCE_INCHES;
  }

  // TODO: Implement system check method
  public Command getElevatorSystemCheckCommand() {
    return null;
  }

  public void goToPosition(ScoringHeight reefBranch) {
    targetPosition = reefBranch;
    elevatorIO.setPosition(reefBranchToDistance(reefBranch));
  }

  public Distance getPosition() {
    return Inches.of(inputs.positionInches);
  }

  private ScoringHeight getSelectedPosition() {
    if (OISelector.getOperatorInterface().getLevel4Trigger().getAsBoolean()) {
      return ScoringHeight.L4;
    } else if (OISelector.getOperatorInterface().getLevel3Trigger().getAsBoolean()) {
      return ScoringHeight.L3;
    } else if (OISelector.getOperatorInterface().getLevel2Trigger().getAsBoolean()) {
      return ScoringHeight.L2;
    } else {
      return ScoringHeight.L1;
    }
  }

  public void goToSelectedPosition() {
    goToPosition(getSelectedPosition());
  }

  public boolean canScoreFartherAway() {
    return Math.abs(distanceFromReef.getX()) < FAR_SCORING_DISTANCE
        && Math.abs(distanceFromReef.getX()) > DrivetrainConstants.DRIVE_TO_REEF_X_TOLERANCE
        && Math.abs(distanceFromReef.getY()) < FAR_SCORING_Y_TOLERANCE
        && Math.abs(distanceFromReef.getRotation().getDegrees())
            < FAR_SCORING_THETA_TOLERANCE.getDegrees();
  }

  public void setDistanceFromReef(Transform2d distance) {
    distanceFromReef = distance;
  }

  public double getXFromReef() {
    return distanceFromReef.getX();
  }

  public void setXFromReef(double x) {
    distanceFromReef = new Transform2d(x, distanceFromReef.getY(), distanceFromReef.getRotation());
  }

  public boolean isAtSelectedPosition() {
    return isAtPosition(getSelectedPosition());
  }

  public void goBelowNearestAlgae() {
    AlgaePosition nearestAlgae = Field2d.getInstance().getNearestAlgae();
    if (nearestAlgae.isHigh) {
      goToPosition(ScoringHeight.BELOW_HIGH_ALGAE);
    } else {
      goToPosition(ScoringHeight.BELOW_LOW_ALGAE);
    }
  }

  public boolean isBelowNearestAlgae() {
    AlgaePosition nearestAlgae = Field2d.getInstance().getNearestAlgae();
    if (nearestAlgae.isHigh) {
      return isAtPosition(ScoringHeight.BELOW_HIGH_ALGAE);
    } else {
      return isAtPosition(ScoringHeight.BELOW_LOW_ALGAE);
    }
  }

  public void goToNearestAlgae() {
    AlgaePosition nearestAlgae = Field2d.getInstance().getNearestAlgae();
    if (nearestAlgae.isHigh) {
      goToPosition(ScoringHeight.HIGH_ALGAE);
    } else {
      goToPosition(ScoringHeight.LOW_ALGAE);
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
        Commands.runOnce(() -> goToPosition(ScoringHeight.HARDSTOP)),
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
