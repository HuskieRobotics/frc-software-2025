package frc.robot.subsystems.subsystem.manipulator.manipulator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.subsystem.manipulator.manipulator.ManipulatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTunableNumber;
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

  private final LoggedTunableNumber indexerMotorCurrent =
      new LoggedTunableNumber("Manipulator/Indexer/MotorCurrent", 0);

  Timer coralInIndexingState =
      new Timer(); // create a timer to track how long is spent in this stage

  private ManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs =
      new ManipulatorIOInputsAutoLogged(); // stefan said to ignore this error and keep this here
  private State state = State.WAITING_FOR_CORAL_IN_FUNNEL;
  private State lastState = State.UNINITIALIZED;

  private LinearFilter currentInAmps =
      LinearFilter.singlePoleIIR(
          0.1, 0.02); // the first value is the time constant, the characteristic timescale of the
  // filter's impulse response, and the second value is the time-period, how often
  // the calculate() method will be called

  private boolean shootCoralButtonPressed = false;
  private boolean removeAlgaeButtonPressed = false;

  private boolean algaeRemoved =
      false; // need to actually figure out the IO stuff with this... how do we actually know if the
  // algae has been removed??

  /**
   * Create a new subsystem with its associated hardware interface object.
   *
   * @param io the hardware interface object for this subsystem
   */
  public Manipulator(ManipulatorIO io) {

    this.io = io;

    SysIdRoutineChooser.getInstance().addOption("Manipulator Current", sysIDManipulator);

    SysIdRoutineChooser.getInstance().addOption("Indexer Current", sysIDIndexer);

    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  /* SysId routine for characterizing the subsystem. This is used to find FF/PID gains for the motor. */
  // for the SysIdRoutine, i added replaced the line where it calls the setMotorVoltage method, and
  // added 2 lines that do the same thing but set the indexer motor voltage, and the funnel motor
  // voltage, using their respective methods

  // sysID for manipulator in current
  private final SysIdRoutine sysIDManipulator =
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
    WAITING_FOR_CORAL_IN_FUNNEL {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setFunnelMotorVelocity(
            FUNNEL_MOTOR_VELOCITY_WHILE_COLLECTING_CORAL); // velocity is tbd
        subsystem.setIndexerMotorVelocity(0); // turn indexer motor speed to 0
      }

      @Override
      void execute(Manipulator subsystem) {
        if (subsystem.inputs.isFunnelIRBlocked) { // ignore the error on this line
          subsystem.setState(State.INDEXING_CORAL_IN_MANIPULATOR);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {
        /*NO-OP */
      }
    },
    INDEXING_CORAL_IN_MANIPULATOR {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setIndexerMotorVelocity(
            INDEXER_MOTOR_VELOCITY_WHILE_COLLECTING_CORAL); // velocity is tbd
        subsystem.coralInIndexingState.restart(); // start timer
      }

      @Override
      void execute(Manipulator subsystem) {
        if (subsystem.inputs.isIndexerIRBlocked
            && subsystem.currentInAmps.lastValue()
                > THESHOLD_FOR_CURRENT_SPIKE) // the currentInAmps filters out the current in the
        // noise and getting the lastValue gets the last value
        // of the current, and if that last value is greater
        // than some constant, then current spike has been
        // detected
        {
          subsystem.setState(State.CORAL_IN_MANIPULATOR);
        } else if (subsystem.coralInIndexingState.hasElapsed(CORAL_COLLECTION_TIME_OUT)
            && subsystem
                .inputs
                .isFunnelIRBlocked) // hasElapsed method check if the timer has elapsed a certain
        // number of seconds, which i can make a constant later
        {
          subsystem.setState(CORAL_STUCK);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {
        subsystem.setFunnelMotorVelocity(
            0); // turn off the funnel motor, regardless of if it is going to the CORAL_STUCK state
        // or the CORAL_IN_MANIPULATOR state
      }
    },
    CORAL_STUCK {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setFunnelMotorVelocity(
            FUNNEL_MOTOR_VELOCITY_WHILE_EJECTING_CORAL); // set negative velocity to funnel motor to
        // invert it
        subsystem.setIndexerMotorVelocity(
            INDEXER_MOTOR_VELOCITY_WHILE_EJECTING_CORAL); // set negative velocity to indexer motor
        // to invert it
      }

      @Override
      void execute(Manipulator subsystem) {
        if (!subsystem.inputs.isFunnelIRBlocked && !subsystem.inputs.isIndexerIRBlocked) {
          subsystem.setState(State.WAITING_FOR_CORAL_IN_FUNNEL);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {
        /*NO-OP */
      }
    },
    CORAL_IN_MANIPULATOR {
      @Override
      void onEnter(Manipulator subsystem) {
        // set both motor speeds to 0
        subsystem.setIndexerMotorVelocity(0);
        subsystem.setFunnelMotorVelocity(0);
      }

      @Override
      void execute(Manipulator subsystem) {
        if (subsystem.shootCoralButtonPressed) {
          subsystem.setState(State.SHOOT_CORAL);
          subsystem.shootCoralButtonPressed = false;
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
        subsystem.setIndexerMotorVelocity(
            INDEXER_MOTOR_VELOCITY_WHILE_SHOOTING_CORAL); // speed of indexer motor velocity while
        // shooting coral should be different
        // compared to intaking, etc
      }

      @Override
      void execute(Manipulator subsystem) {
        // call command to shoot coral -- will do later
        if ((!subsystem.inputs.isFunnelIRBlocked && !subsystem.inputs.isIndexerIRBlocked)
            && subsystem.removeAlgaeButtonPressed) {
          subsystem.setState(State.REMOVE_ALGAE);
          subsystem.removeAlgaeButtonPressed = false;
        } else if (!subsystem.inputs.isFunnelIRBlocked && !subsystem.inputs.isIndexerIRBlocked) {
          subsystem.setState(
              State.WAITING_FOR_CORAL_IN_FUNNEL); // if the coral has been shot out and the
          // manipulator is empty
        }
      }

      @Override
      void onExit(Manipulator subsystem) {
        // set speed of indexer motor to 0
        subsystem.setIndexerMotorVelocity(0);
      }
    },
    REMOVE_ALGAE {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setIndexerMotorVelocity(INDEXER_MOTOR_VELOCITY_WHILE_REMOVING_ALGAE);
      }

      @Override
      void execute(Manipulator subsystem) {
        if (subsystem
            .algaeRemoved) // I created an instance variable (algaeRemoved) thqat would keep track
        // if the algae was removed but I need to do the IO stuff to actually
        // check if the algae has been removed
        {
          subsystem.setState(State.WAITING_FOR_CORAL_IN_FUNNEL);
          subsystem.algaeRemoved = false;
        }
      }

      @Override
      void onExit(Manipulator subsystem) {
        /*NO-OP */
      }
    },
    UNINITIALIZED { // state that the robot should be in when its  turned off
      @Override
      void onEnter(Manipulator subsystem) {
        // set speed of both motors to 0
        subsystem.setIndexerMotorVelocity(0);
        subsystem.setFunnelMotorVelocity(0);
      }

      @Override
      void execute(Manipulator subsystem) {
        subsystem.setState(State.WAITING_FOR_CORAL_IN_FUNNEL);
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

      if (indexerMotorVoltage.get() != 0) {
        setIndexerMotorVoltage(indexerMotorVoltage.get());
      } else if (indexerMotorVelocity.get() != 0) {
        setIndexerMotorVelocity(indexerMotorVelocity.get());
      } else if (indexerMotorCurrent.get() != 0) {
        setIndexerMotorCurrent(indexerMotorCurrent.get());
      }
    } else {
      runStateMachine();
    }
  }

  private void setState(State state) {
    this.state = state;
  }

  private void runStateMachine() {
    if (state != lastState) {
      lastState.onExit(this);
      lastState = state;
      state.onEnter(this);
    }

    state.execute(this);
  }

  /**
   * Set the motor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the motor to
   */
  public void setFunnelMotorVoltage(double volts) {
    io.setFunnelMotorVoltage(volts);
  }

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  public void setFunnelMotorCurrent(double current) {
    io.setFunnelMotorCurrent(current);
  }

  /**
   * Set the motor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the motor to
   */
  public void setIndexerMotorVoltage(double volts) {
    io.setIndexerMotorVoltage(volts);
  }

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  public void setIndexerMotorCurrent(double current) {
    io.setIndexerMotorCurrent(current);
  }

  public void setFunnelMotorVelocity(double velocity) {
    io.setFunnelMotorVelocity(velocity);
  }

  public void setIndexerMotorVelocity(double velocity) {
    io.setIndexerMotorVelocity(velocity);
  }

  // Whichever line of code does something with the motors, i replaced it with 2 lines that do the
  // same exact thing but for the funnel and indexer motor, unsure if this is correct
  private Command getSystemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> FaultReporter.getInstance().clearFaults(SUBSYSTEM_NAME)),
            Commands.run(() -> io.setFunnelMotorVoltage(3.6)).withTimeout(1.0),
            Commands.run(() -> io.setIndexerMotorVoltage(3.6)).withTimeout(1.0),
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
            Commands.run(() -> io.setFunnelMotorVoltage(-2.4)).withTimeout(1.0),
            Commands.run(() -> io.setIndexerMotorVoltage(-2.4)).withTimeout(1.0),
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
        .andThen(Commands.runOnce(() -> io.setFunnelMotorVoltage(0.0)))
        .andThen(Commands.runOnce(() -> io.setIndexerMotorVoltage(0.0)));
  }

  // method to shoot coral which assigns coral  button presed to true
  public void shootCoral() {
    shootCoralButtonPressed = true;
  }

  // method to remove algae which assins the remove algae button pressed to true
  public void removeAlgae() {
    removeAlgaeButtonPressed = true;
  }
}
