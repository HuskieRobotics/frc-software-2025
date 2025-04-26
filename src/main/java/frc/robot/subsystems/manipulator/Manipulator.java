package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.leds.LEDs.States;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.operator_interface.OISelector;
import org.littletonrobotics.junction.Logger;

/**
 * Models a generic subsystem for a rotational mechanism. The other subsystems defined in this
 * library aren't good examples for typical robot subsystems. This class can serve as an example or
 * be used for quick prototyping.
 */
public class Manipulator extends SubsystemBase {

  // these Tunables are convenient when testing as they provide direct control of the subsystem's
  // motor
  private final LoggedTunableNumber testingMode =
      new LoggedTunableNumber("Manipulator/TestingMode", 0);

  private final LoggedTunableNumber funnelMotorVoltage =
      new LoggedTunableNumber("Manipulator/Funnel/MotorVoltage", 0);

  private final LoggedTunableNumber funnelMotorVelocity =
      new LoggedTunableNumber("Manipulator/Funnel/MotorVelocity", 0);

  private final LoggedTunableNumber indexerMotorVoltage =
      new LoggedTunableNumber("Manipulator/Indexer/MotorVoltage", 0);

  private final LoggedTunableNumber indexerMotorVelocity =
      new LoggedTunableNumber("Manipulator/Indexer/MotorVelocity", 0);

  private final LoggedTunableNumber funnelMotorCurrent =
      new LoggedTunableNumber("Manipulator/Funnel/MotorCurrent", 0);

  private final LoggedTunableNumber indexerHoldAlgaeCurrent =
      new LoggedTunableNumber("Manipulator/Indexer/MotorCurrent", INDEXER_HOLD_ALGAE_CURRENT);

  public final LoggedTunableNumber indexerCollectionVoltage =
      new LoggedTunableNumber("Manipulator/Indexer/CollectionVoltage", INDEXER_COLLECTION_VOLTAGE);

  public final LoggedTunableNumber fastShootingVoltage =
      new LoggedTunableNumber(
          "Manipulator/Indexer/Level1And4ShootingVoltage", INDEXER_SHOOT_FAST_VOLTAGE);

  public final LoggedTunableNumber slowShootingVoltage =
      new LoggedTunableNumber(
          "Manipulator/Indexer/Level2And3ShootingVoltage", INDEXER_SHOOT_SLOW_VOLTAGE);

  public final LoggedTunableNumber indexerEjectingVoltage =
      new LoggedTunableNumber("Manipulator/Indexer/EjectingVoltage", INDEXER_EJECT_VOLTAGE);
  public final LoggedTunableNumber funnelCollectionVoltage =
      new LoggedTunableNumber("Manipulator/Funnel/CollectionVoltage", FUNNEL_COLLECTION_VOLTAGE);
  public final LoggedTunableNumber funnelEjectingVoltage =
      new LoggedTunableNumber("Manipulator/Funnel/EjectingVoltage", FUNNEL_EJECT_VOLTAGE);

  private final LoggedTunableNumber pivotAngle =
      new LoggedTunableNumber("Manipulator/Pivot/angle", 0); // add angle

  private final LoggedTunableNumber pivotMotorVoltage =
      new LoggedTunableNumber("Manipulator/Pivot/MotorVoltage", 0);

  private final LoggedTunableNumber pivotMotorCurrent =
      new LoggedTunableNumber("Manipulator/Pivot/Current", 0);

  Timer coralInIndexingState =
      new Timer(); // create a timer to track how long is spent in this stage

  Timer ejectingCoralTimer = new Timer();
  Timer intakingAlgaeTimer = new Timer();
  Timer scoringAlgaeTimer = new Timer();

  private ManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();
  private State state = State.WAITING_FOR_CORAL;
  private State lastState = State.UNINITIALIZED;

  private LinearFilter currentInAmps =
      LinearFilter.singlePoleIIR(
          0.1, 0.02); // the first value is the time constant, the characteristic timescale of the
  // filter's impulse response, and the second value is the time-period, how often
  // the calculate() method will be called

  private boolean shootCoralButtonPressed = false;
  private boolean intakeAlgaeButtonPressed = false;
  private boolean scoreAlgaeInBargeButtonPressed = false;
  private boolean scoreAlgaeInProcessorButtonPressed = false;
  private boolean dropAlgaeButtonPressed = false;

  private boolean readyToScore = false;
  private boolean shootingFast = false;

  private boolean disableFunnelForClimb = false;
  private double targetIndexerPosition;

  private boolean coralInManipulatorFirstRun = true;

  /**
   * Create a new subsystem with its associated hardware interface object.
   *
   * @param io the hardware interface object for this subsystem
   */
  public Manipulator(ManipulatorIO io) {

    this.io = io;

    SysIdRoutineChooser.getInstance()
        .addOption(
            "Funnel Current",
            sysIDFunnel); // changed name from "Manipulator Current" and sysIDManipulator to "Funnel
    // Current" and sysIDFunnel

    SysIdRoutineChooser.getInstance().addOption("Indexer Current", sysIDIndexer);
    // SysIdRoutineChooser.getInstance().addOption("Pivot Voltage", sysIdPivot);

    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  /* SysId routine for characterizing the subsystem. This is used to find FF/PID gains for the motor. */
  // for the SysIdRoutine, i added replaced the line where it calls the setMotorVoltage method, and
  // added 2 lines that do the same thing but set the indexer motor voltage, and the funnel motor
  // voltage, using their respective methods

  // sysID for manipulator in current
  private final SysIdRoutine sysIDFunnel = // changed name from SysIDManipulator to SysIDFunnel
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              null, // Use default step voltage (7 V)
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              sysIDState -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setFunnelMotorCurrent(output.in(Volts)), null, this));

  private final SysIdRoutine sysIDIndexer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              null, // Use default step voltage (7 V)
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              sysIDState -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setIndexerMotorCurrent(output.in(Volts)), null, this));

  // private final SysIdRoutine sysIdPivot =
  //     new SysIdRoutine( // FIXME: check values for ramp rate and step voltage
  //         new SysIdRoutine.Config(
  //             Volts.of(0.1).per(Seconds), // Use default ramp rate (1 V/s)
  //             Volts.of(0.6), // Use default step voltage (7 V)
  //             null, // Use default timeout (10 s)
  //             // Log state with SignalLogger class
  //             sysIDState -> SignalLogger.writeString("SysId_State", state.toString())),
  //         new SysIdRoutine.Mechanism(output -> setPivotMotorVoltage(output.in(Volts)), null,
  // this));

  /**
   * Few subsystems require the complexity of a state machine. A simpler command-based approach is
   * usually better. However, there are times when diagraming and implementing a formal state
   * machine is a reasonable approach. This code is designed to facilitate mapping from a formal
   * state machine diagram to code.
   *
   * <p>The state machine is defined as an enum with each state having its own execute, onEnter, and
   * onExit methods. The execute method is called every iteration of the periodic method. The
   * onEnter and onExit methods are called when the state is entered and exited, respectively.
   * Transitions between states are defined in the execute methods. It is critical that the setState
   * method is only invoked within a state's execute method. Otherwise, it is possible for a state
   * transition to be missed.
   *
   * <p>This approach is modeled after this ChiefDelphi post:
   * https://www.chiefdelphi.com/t/enums-and-subsytem-states/463974/6
   */
  private enum State {
    WAITING_FOR_CORAL {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setFunnelMotorVoltage(subsystem.funnelCollectionVoltage.get());
        subsystem.setIndexerMotorVoltage(subsystem.indexerCollectionVoltage.get());
        subsystem.readyToScore = false;
      }

      @Override
      void execute(Manipulator subsystem) {

        LEDs.getInstance().requestState(States.WAITING_FOR_CORAL);
        subsystem.retractPivot();

        if (subsystem.disableFunnelForClimb) {
          subsystem.setFunnelMotorVoltage(0.0);
          subsystem.disableFunnelForClimb =
              false; // set to false so we don't periodically request 0 voltage
        }

        if (subsystem.inputs.isFunnelIRBlocked) {
          subsystem.setState(State.INDEXING_CORAL_IN_MANIPULATOR);
        } else if (DriverStation.isDisabled() && subsystem.inputs.isIndexerIRBlocked) {
          subsystem.setState(State.CORAL_IN_MANIPULATOR);
        } else if (subsystem.inputs.isIndexerIRBlocked) {
          subsystem.setState(State.INDEXING_CORAL_IN_MANIPULATOR);
        } else if (subsystem.intakeAlgaeButtonPressed) {
          subsystem.setState(State.WAITING_FOR_ALGAE);
        } else if (subsystem.inputs.isAlgaeIRBlocked) {
          subsystem.setState(State.ALGAE_IN_MANIPULATOR);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },
    INDEXING_CORAL_IN_MANIPULATOR {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setFunnelMotorVoltage(subsystem.funnelCollectionVoltage.get());
        subsystem.setIndexerMotorVoltage(subsystem.indexerCollectionVoltage.get());
        subsystem.coralInIndexingState.restart(); // start timer
        subsystem.currentInAmps
            .reset(); // reset the linear filter thats used to detect a current spike
      }

      @Override
      void execute(Manipulator subsystem) {

        LEDs.getInstance().requestState(States.INDEXING_CORAL);
        subsystem.retractPivot();

        if (subsystem.inputs.isIndexerIRBlocked
            && subsystem.currentInAmps.lastValue()
                > CORAL_CURRENT_SPIKE_THRESHOLD) // the currentInAmps filters out the current in
        // the
        // noise and getting the lastValue gets the last value
        // of the current, and if that last value is greater
        // than some constant, then current spike has been
        // detected
        {
          subsystem.setState(State.CORAL_IN_MANIPULATOR);
        } else if (subsystem.coralInIndexingState.hasElapsed(
            CORAL_COLLECTION_TIME_OUT)) // hasElapsed method check if the timer has elapsed a
        // certain number of seconds
        {
          subsystem.setState(CORAL_STUCK);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },
    CORAL_STUCK {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setFunnelMotorVoltage(
            subsystem.funnelEjectingVoltage.get()); // set negative velocity to funnel motor to
        // invert it
        subsystem.setIndexerMotorVoltage(
            subsystem.indexerEjectingVoltage.get()); // set negative velocity to indexer motor
        // to invert it
        subsystem.ejectingCoralTimer.restart();
      }

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.EJECTING_CORAL);
        subsystem.retractPivot();

        if (subsystem.inputs.isIndexerIRBlocked) {
          subsystem.setState(State.CORAL_IN_MANIPULATOR);
        } else if (!subsystem.inputs.isFunnelIRBlocked
            && !subsystem.inputs.isIndexerIRBlocked
            && subsystem.ejectingCoralTimer.hasElapsed(FINAL_EJECT_CORAL_DURATION_SECONDS)) {
          subsystem.setState(State.WAITING_FOR_CORAL);
        } else if (subsystem.ejectingCoralTimer.hasElapsed(SECOND_INTAKE_CORAL_SECONDS)) {
          subsystem.setFunnelMotorVoltage(subsystem.funnelEjectingVoltage.get());
          subsystem.setIndexerMotorVoltage(subsystem.indexerEjectingVoltage.get());
        } else if (subsystem.ejectingCoralTimer.hasElapsed(FIRST_EJECT_CORAL_SECONDS)) {
          subsystem.setFunnelMotorVoltage(subsystem.funnelCollectionVoltage.get());
          subsystem.setIndexerMotorVoltage(subsystem.indexerCollectionVoltage.get());
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },
    CORAL_IN_MANIPULATOR {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setFunnelMotorVoltage(0.0);
        subsystem.zeroIndexerPosition();
        subsystem.targetIndexerPosition = 0.0;
        subsystem.shootCoralButtonPressed = false;
        subsystem.coralInManipulatorFirstRun = true;
      }

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.HAS_CORAL);
        subsystem.retractPivot();

        if (DriverStation.isDisabled()) {
          subsystem.targetIndexerPosition = subsystem.inputs.indexerPositionRotations;
        }

        if (subsystem.coralInManipulatorFirstRun) {
          subsystem.coralInManipulatorFirstRun = false;
        } else {
          subsystem.holdWheelPosition(subsystem.targetIndexerPosition);
        }

        Logger.recordOutput(
            SUBSYSTEM_NAME + "/targetWheelPosition", subsystem.targetIndexerPosition);

        if (subsystem.shootCoralButtonPressed) {
          subsystem.setState(State.SHOOT_CORAL);
          subsystem.shootCoralButtonPressed = false;
        } else if (!subsystem.inputs.isIndexerIRBlocked) {
          subsystem.setState(State.WAITING_FOR_CORAL);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {
        /*NO-OP */
      }
    },
    SHOOT_CORAL {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.readyToScore = false;

        if (subsystem.shootingFast) {
          if (OISelector.getOperatorInterface().getLevel2Trigger().getAsBoolean()
              || OISelector.getOperatorInterface().getLevel3Trigger().getAsBoolean()) {
            subsystem.setIndexerMotorVoltage(subsystem.slowShootingVoltage.get());
          } else {
            subsystem.setIndexerMotorVoltage(subsystem.fastShootingVoltage.get());
          }
        } else {
          subsystem.setIndexerMotorVoltage(subsystem.fastShootingVoltage.get());
        }
      }

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.SCORING);
        subsystem.retractPivot();

        if (!subsystem.inputs.isFunnelIRBlocked && !subsystem.inputs.isIndexerIRBlocked) {
          subsystem.setState(State.WAITING_FOR_CORAL);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },
    WAITING_FOR_ALGAE {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setPivotMotorCurrent(PIVOT_EXTEND_CURRENT);
        subsystem.setFunnelMotorVoltage(0.0);
        subsystem.setIndexerMotorVoltage(INDEXER_COLLECT_ALGAE_VOLTAGE);
        subsystem.intakingAlgaeTimer.restart();
        subsystem.currentInAmps.reset();
      }

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.COLLECTING_ALGAE);

        // check for current spike or if algae IR has detected algae
        if (subsystem.inputs.isAlgaeIRBlocked
            && subsystem.currentInAmps.lastValue() > ALGAE_CURRENT_SPIKE_THRESHOLD) {
          subsystem.setState(State.ALGAE_IN_MANIPULATOR);
        } else if (subsystem.intakingAlgaeTimer.hasElapsed(INTAKE_ALGAE_TIMEOUT)) {
          subsystem.setState(State.WAITING_FOR_CORAL);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {
        // set the boolean that controls if algae intake button has been pressed to false
        subsystem.intakeAlgaeButtonPressed = false;
      }
    },

    ALGAE_IN_MANIPULATOR {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setPivotMotorCurrent(0.0);
        subsystem.setFunnelMotorVoltage(0.0);
        subsystem.setIndexerMotorCurrent(subsystem.indexerHoldAlgaeCurrent.get());
      }

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.HAS_ALGAE);

        // check if the shootAlgae button has been pressed, if so then switch to the SHOOT_ALGAE
        // state
        if (subsystem.scoreAlgaeInBargeButtonPressed) {
          subsystem.setState(State.SHOOT_ALGAE_IN_BARGE);
        } else if (subsystem.scoreAlgaeInProcessorButtonPressed) {
          subsystem.setState(State.SHOOT_ALGAE_IN_PROCESSOR);
        } else if (subsystem.dropAlgaeButtonPressed) {
          subsystem.setState(State.DROP_ALGAE);
        } else if (!subsystem.inputs.isAlgaeIRBlocked) {
          subsystem.setState(State.WAITING_FOR_CORAL);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },
    SHOOT_ALGAE_IN_BARGE { // state robot is in while algae is being shot out of the manipulator
      @Override
      void onEnter(Manipulator subsystem) {
        // set the indexer/roller motor to a negative voltage in order for the rollers to move the
        // opp direction and eject the algae out of the manipulator

        subsystem.setFunnelMotorVoltage(0.0);
        subsystem.setIndexerMotorCurrent(INDEXER_SHOOT_ALGAE_BARGE_CURRENT);
        subsystem.scoreAlgaeInBargeButtonPressed = false;
        subsystem.setPivotMotorCurrent(0.0);

        subsystem.scoringAlgaeTimer.restart();
      }

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.SCORING);

        if (!subsystem.inputs.isAlgaeIRBlocked
            && subsystem.scoringAlgaeTimer.hasElapsed(BARGE_ALGAE_TIMEOUT)) {
          subsystem.setState(State.WAITING_FOR_CORAL);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },
    SHOOT_ALGAE_IN_PROCESSOR {
      @Override
      void onEnter(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.SCORING);

        subsystem.setFunnelMotorVoltage(0.0);
        subsystem.setIndexerMotorCurrent(INDEXER_SHOOT_ALGAE_PROCESSOR_CURRENT);
        subsystem.scoreAlgaeInBargeButtonPressed = false;
        subsystem.scoreAlgaeInProcessorButtonPressed = false;
        subsystem.setPivotMotorCurrent(0.0);

        subsystem.scoringAlgaeTimer.restart();
      }

      @Override
      void execute(Manipulator subsystem) {
        if (!subsystem.inputs.isAlgaeIRBlocked
            && subsystem.scoringAlgaeTimer.hasElapsed(PROCESSOR_ALGAE_TIMEOUT)) {
          subsystem.setState(State.WAITING_FOR_CORAL);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },
    DROP_ALGAE {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setFunnelMotorVoltage(0.0);
        subsystem.setIndexerMotorCurrent(INDEXER_DROP_ALGAE_CURRENT);
        subsystem.dropAlgaeButtonPressed = false;
        subsystem.setPivotMotorCurrent(0.0);

        subsystem.scoringAlgaeTimer.restart();
      }

      @Override
      void execute(Manipulator subsystem) {
        if (!subsystem.inputs.isAlgaeIRBlocked
            && subsystem.scoringAlgaeTimer.hasElapsed(DROP_ALGAE_TIMEOUT)) {
          subsystem.setState(State.WAITING_FOR_CORAL);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },

    UNINITIALIZED {

      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setFunnelMotorVoltage(0);
        subsystem.setIndexerMotorVoltage(0);
        subsystem.setPivotMotorCurrent(0);
      }

      @Override
      void execute(Manipulator subsystem) {
        subsystem.setState(
            State.WAITING_FOR_CORAL); // default state to WAITING_FOR_CORAL_IN_FUNNEL state
      }

      @Override
      void onExit(Manipulator subsystem) {
        /*NO-OP */
      }
    };

    abstract void execute(Manipulator subsystem);

    abstract void onEnter(Manipulator subsystem);

    abstract void onExit(Manipulator subsystem);
  }

  /**
   * The subsystem's periodic method needs to update and process the inputs from the hardware
   * interface object.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    Logger.recordOutput(SUBSYSTEM_NAME + "/State", this.state);
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/scoreThroughManipulatorPressed", shootCoralButtonPressed);
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/scoreAlgaeInBargeButtonPressed", scoreAlgaeInBargeButtonPressed);
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/scoreAlgaeInProcessorButtonPressed", scoreAlgaeInProcessorButtonPressed);
    Logger.recordOutput(SUBSYSTEM_NAME + "/dropAlgaeButtonPressed", dropAlgaeButtonPressed);
    currentInAmps.calculate(inputs.indexerStatorCurrentAmps);

    // when testing, set the FUNNEL motor power, current, or position based on the Tunables (if
    // non-zero)
    // need voltage, current, and velocity

    if (testingMode.get() == 1) {
      if (funnelMotorVoltage.get() != 0) {
        setFunnelMotorVoltage(funnelMotorVoltage.get());
      } else if (funnelMotorVelocity.get() != 0) {
        setFunnelMotorVelocity(funnelMotorVelocity.get());
      } else if (funnelMotorCurrent.get() != 0) {
        setFunnelMotorCurrent(funnelMotorCurrent.get());
      }

      // set the indexer motor power, current, or position based on the Tunables (if non-zero)
      if (indexerMotorVoltage.get() != 0) {
        setIndexerMotorVoltage(indexerMotorVoltage.get());
      } else if (indexerMotorVelocity.get() != 0) {
        setIndexerMotorVelocity(indexerMotorVelocity.get());
      } else if (indexerHoldAlgaeCurrent.get() != 0) {
        setIndexerMotorCurrent(indexerHoldAlgaeCurrent.get());
      }

      // if (pivotAngle.get() != 0) {
      //   setPivotPosition(Degrees.of(pivotAngle.get()));
      // } else if (pivotMotorVoltage.get() != 0) {
      //   setPivotMotorVoltage(pivotMotorVoltage.get());
      // }

    } else {
      runStateMachine();
    }

    // Record cycle time
    LoggedTracer.record("Manipulator");
  }

  private void setState(State state) {
    this.state = state;
  }

  public void resetStateMachine() {
    this.state = State.WAITING_FOR_CORAL;
  }

  private void runStateMachine() {
    if (state != lastState) {
      lastState.onExit(this);
      lastState = state;
      state.onEnter(this);
    }

    state.execute(this);
  }

  public void disableFunnelForClimb() {
    this.disableFunnelForClimb = true;
  }

  public void setFunnelMotorVoltage(double volts) {
    io.setFunnelVoltage(volts);
  }

  public void setFunnelMotorCurrent(double current) {
    io.setFunnelCurrent(current);
  }

  public void setFunnelMotorVelocity(double velocity) {
    io.setFunnelVelocity(velocity);
  }

  public void setIndexerMotorVoltage(double volts) {
    io.setIndexerVoltage(volts);
  }

  public void zeroIndexerPosition() {
    io.zeroIndexerPosition();
  }

  public void setIndexerMotorCurrent(double current) {
    io.setIndexerCurrent(current);
  }

  public void setIndexerMotorVelocity(double velocity) {
    io.setIndexerVelocity(velocity);
  }

  public void setPivotMotorCurrent(double current) {
    io.setPivotCurrent(current);
  }

  public void holdWheelPosition(double targetPosition) {
    io.setIndexerPosition(targetPosition);
  }

  // Whichever line of code does something with the motors, i replaced it with 2 lines that do the
  // same exact thing but for the funnel and indexer motor, unsure if this is correct
  private Command getSystemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> FaultReporter.getInstance().clearFaults(SUBSYSTEM_NAME)),
            Commands.run(() -> io.setFunnelVoltage(3.6)).withTimeout(1.0),
            Commands.run(() -> io.setIndexerVoltage(3.6)).withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.funnelVelocityRPS < 2.0) {
                    FaultReporter.getInstance()
                        .addFault(
                            SUBSYSTEM_NAME,
                            "[System Check] Subsystem motor not moving as fast as expected",
                            false,
                            true);
                  }
                }),
            Commands.run(() -> io.setFunnelVoltage(-2.4)).withTimeout(1.0),
            Commands.run(() -> io.setIndexerVoltage(-2.4)).withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.indexerVelocityRPS > -2.0) {
                    FaultReporter.getInstance()
                        .addFault(
                            SUBSYSTEM_NAME,
                            "[System Check] Subsystem motor moving too slow or in the wrong direction",
                            false,
                            true);
                  }
                }))
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(Commands.runOnce(() -> io.setFunnelVoltage(0.0)))
        .andThen(Commands.runOnce(() -> io.setIndexerVoltage(0.0)));
  }

  private void shootCoral() {
    shootCoralButtonPressed = true;
    if (shootingFast) {
      if (OISelector.getOperatorInterface().getLevel2Trigger().getAsBoolean()
          || OISelector.getOperatorInterface().getLevel3Trigger().getAsBoolean()) {
        setIndexerMotorVoltage(slowShootingVoltage.get());
      } else {
        setIndexerMotorVoltage(fastShootingVoltage.get());
      }
    } else {
      setIndexerMotorVoltage(fastShootingVoltage.get());
    }
  }

  public void shootCoralFast() {
    shootingFast = true;
    shootCoral();
  }

  public void shootCoralSlow() {
    shootingFast = false;
    shootCoral();
  }

  public void retractPivot() {
    if (inputs.pivotMotorAngleDeg > 85.0) {
      setPivotMotorCurrent(PIVOT_RETRACT_HOLD_CURRENT);
    } else {
      setPivotMotorCurrent(PIVOT_RETRACT_UP_CURRENT);
    }
  }

  public void scoreAlgaeInBarge() {
    scoreAlgaeInBargeButtonPressed = true;
    scoreAlgaeInProcessorButtonPressed = false;
    dropAlgaeButtonPressed = false;
  }

  public void scoreAlgaeInProcessor() {
    scoreAlgaeInProcessorButtonPressed = true;
    scoreAlgaeInBargeButtonPressed = false;
    dropAlgaeButtonPressed = false;
  }

  public void dropAlgae() {
    dropAlgaeButtonPressed = true;
    scoreAlgaeInBargeButtonPressed = false;
    scoreAlgaeInProcessorButtonPressed = false;
  }

  public boolean doneCollectingAlgae() {
    return state == State.ALGAE_IN_MANIPULATOR || state == State.WAITING_FOR_CORAL;
  }

  public boolean indexingCoral() {
    return state == State.INDEXING_CORAL_IN_MANIPULATOR;
  }

  public boolean hasIndexedCoral() {
    return state == State.CORAL_IN_MANIPULATOR;
  }

  public boolean hasIndexedAlgae() {
    return state == State.ALGAE_IN_MANIPULATOR;
  }

  public boolean coralIsInManipulator() {
    return inputs.isIndexerIRBlocked;
  }

  public boolean algaeIsInManipulator() {
    return inputs.isAlgaeIRBlocked;
  }

  public boolean scoredAlgae() {
    return state == State.WAITING_FOR_CORAL;
  }

  public void setReadyToScore(boolean readyToScore) {
    this.readyToScore = readyToScore;
  }

  public void collectAlgae() {
    intakeAlgaeButtonPressed = true;
  }

  public boolean isReadyToScore() {
    return readyToScore;
  }

  public Angle getPivotAngle() {
    return Degrees.of(inputs.pivotMotorAngleDeg);
  }
}
