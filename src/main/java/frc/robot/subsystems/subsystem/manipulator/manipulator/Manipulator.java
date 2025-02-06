package frc.robot.subsystems.subsystem.manipulator.manipulator;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.subsystem.manipulator.manipulator.ManipulatorConstants.*;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.LinearFilter;

/**
 * Models a generic subsystem for a rotational mechanism. The other subsystems defined in this
 * library aren't good examples for typical robot subsystems. This class can serve as an example or
 * be used for quick prototyping.
 */
public class Manipulator extends SubsystemBase {

  // these Tunables are convenient when testing as they provide direct control of the subsystem's
  // motor
  private final LoggedTunableNumber testingMode =
      new LoggedTunableNumber("Subsystem/TestingMode", 0);

//logged tunable numbers for funnel motor stuff
  private final LoggedTunableNumber funnelMotorPower = 
      new LoggedTunableNumber("Subsystem/power", 0.0);
  private final LoggedTunableNumber funnelMotorCurrent =
      new LoggedTunableNumber("Subsystem/current", 0.0);
  private final LoggedTunableNumber funnelMotorPosition =
      new LoggedTunableNumber("Subsystem/position", 0.0);

//logged tunable numbers for indexer motor stuff
  private final LoggedTunableNumber indexerMotorPower = 
      new LoggedTunableNumber("Subsystem/power", 0.0);
  private final LoggedTunableNumber indexerMotorCurrent =
      new LoggedTunableNumber("Subsystem/current", 0.0);
  private final LoggedTunableNumber indexerMotorPosition =
      new LoggedTunableNumber("Subsystem/position", 0.0);

  private ManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged(); //stefan said to ignore this error and keep this here
  private State state = State.WAITING_FOR_CORAL_IN_FUNNEL;
  private State lastState = State.UNINITIALIZED; 
  private LinearFilter currentInAmps = new LinearFilter.singlePoleIIR(0.1, 0.02); //the first value is the time constant, the characteristic timescale of the filter's impulse response, and the second value is the time-period, how often the calculate() method will be called
  private final double thresholdForCurrentSpike = 5; //this constant will keep track of the threshold for a current spike , which for example I put as 5, can be changed later
  private boolean shootCoralButtonPressed = false; 
  private boolean removeAlgaeButtonPressed = false;
  private boolean algaeRemoved = false; //need to actually figure out the IO stuff with this... how do we actually know if the algae has been removed??
  //unnecessary --> private boolean robotTurnedOn = false; 
  
  /**
   * Create a new subsystem with its associated hardware interface object.
   *
   * @param io the hardware interface object for this subsystem
   */
  public Manipulator(ManipulatorIO io) {

    this.io = io;

    SysIdRoutineChooser.getInstance().addOption("Manipulator Voltage", sysIdRoutine);

    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

    /* SysId routine for characterizing the subsystem. This is used to find FF/PID gains for the motor. */
    //for the SysIdRoutine, i added replaced the line where it calls the setMotorVoltage method, and added 2 lines that do the same thing but set the indexer motor voltage, and the funnel motor voltage, using their respective methods
    private final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                null, // Use default step voltage (7 V)
                null, // Use default timeout (10 s)
                // Log state with SignalLogger class
                state -> SignalLogger.writeString("SysId_State", state.toString())),
            new SysIdRoutine.Mechanism(io -> setFunnelMotorVoltage(output.in(Volts)), null, this),
            new SysIdRoutine.Mechanism(io -> setIndexerMotorVoltage(output.in(Volts)), null, this));
  
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
       WAITING_FOR_CORAL_IN_FUNNEL{
        @Override
        void onEnter(Manipulator subsystem) {
          subsystem.setFunnelMotorVelocity(3); //velocity is tbd
          subsystem.setIndexerMotorVelocity(0); //turn indexer motor speed to 0 
        }
  
        @Override
        void execute(Manipulator subsystem) {
          if (subsystem.inputs.isFunnelIRBlocked) { //ignore the error on this line
            subsystem.setState(State.INDEXING_CORAL_IN_MANIPULATOR); 
          }
        }
  
        @Override
        void onExit(Manipulator subsystem) {}
      },
      INDEXING_CORAL_IN_MANIPULATOR {
        @Override
        void onEnter(Manipulator subsystem) {
          subsystem.setIndexerMotorVelocity(3); //velocity is tbd
          Timer coralInIndexingState = new Timer(); //create a timer to track how long is spent in this stage
          coralInIndexingState.restart(); //start timer

      }
      @Override
      void execute(Manipulator subsystem) {
        if(subsystem.inputs.isIndexerIRBlocked && currentInAmps.lastValue() > thresholdForCurrentSpike) //the currentInAmps filters out the current in the noise and getting the lastValue gets the last value of the current, and if that last value is greater than some constant, then current spike has been detected
        {
          subsystem.setState(State.CORAL_IN_MANIPULATOR);
        }
        else if (coralInIndexingState.hasElapsed(3) && subsystem.inputs.isFunnelIRBlocked ) //hasElapsed method check if the timer has elapsed a certain number of seconds, which i can make a constant later
        {
          subsystem.setState(CORAL_STUCK);
        }
      }
      @Override
      void onExit(Manipulator subsystem) {
        subsystem.setFunnelMotorVelocity(0); //turn off the funnel motor, regardless of if it is going to the CORAL_STUCK state or the CORAL_IN_MANIPULATOR state
      }
    },
    CORAL_STUCK{
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setFunnelMotorVelocity(-3); //set negative velocity to funnel motor to invert it
        isFunnelMotorInverted = true; //set funnel motor inverted boolean to true
      }
      @Override
      void execute(Manipulator subsystem) {
        if (isFunnelIRBlocked == false && isIndexerIRBlocked == false ) {
          subsystem.setState(State.WAITING_FOR_CORAL_IN_FUNNEL);
        }
      }
      @Override
      void onExit(Manipulator subsystem) {
        isFunnelMotorInverted = false; //set this to false because the robot should move from this state to the WAITING_FOR_CORAL_IN_FUNNEL state, and the funnel motor wouldn't be inverted in that state

      }
    },
    CORAL_IN_MANIPULATOR {
      @Override
      void onEnter(Manipulator subsystem) {
        //set both motor speeds to 0
        subsystem.setIndexerMotorVelocity(0);
        subsystem.setFunnelMotorVelocity(0);
      }

      @Override
      void execute(Manipulator subsystem) {
        if (shootCoralButtonPressed) {
          subsystem.setState(State.SHOOT_CORAL);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {
      }
    },
    SHOOT_CORAL {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setIndexerMotorVelocity(INDEXER_MOTOR_VELOCITY_WHILE_SHOOTING_CORAL); //speed of indexer motor velocity while shooting coral should be different compared to intaking, etc
      }

      @Override
      void execute(Manipulator subsystem) {
        //call command to shoot coral -- will do later
        if ((isFunnelIRBlocked == false && isIndexerIRBlocked == false) && removeAlgaeButtonPressed) {
          subsystem.setState(State.REMOVE_ALGAE); 
        }
        else if (isFunnelIRBlocked == false && isIndexerIRBlocked == false)
        {
          subsystem.setState(State.WAITING_FOR_CORAL_IN_FUNNEL); //if the coral has been shot out and the manipulator is empty
        }
      }

      @Override
      void onExit(Manipulator subsystem) {
        //set speed of indexer motor to 0
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
        if (!removeAlgaeButtonPressed && algaeRemoved ) //I created an instance variable (algaeRemoved) thqat would keep track if the algae was removed but I need to do the IO stuff to actually check if the algae has been removed
        {
          subsystem.setState(State.WAITING_FOR_CORAL_IN_FUNNEL);
        }
      }
      @Override
      void onExit(Manipulator subsystem) {
      }
    },
    UNINITIALIZED { //state that the robot should be in when its  turned off 
      @Override
      void onEnter(Manipulator subsystem) {
        //set speed of both motors to 0
        subsystem.setIndexerMotorVelocity(0);
        subsystem.setFunnelMotorVelocity(0);
      }
      @Override
      void execute(Manipulator subsystem) {
        if(robotTurnedOn)
        {
          subsystem.setState(State.WAITING_FOR_CORAL_IN_FUNNEL);
        }
      }
      @Override
      void onExit(Manipulator subsystem) {
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
    currentInAmps.calculate();

    // when testing, set the FUNNEL motor power, current, or position based on the Tunables (if non-zero)
    if (testingMode.get() != 0) {
      if (funnelMotorPower.get() != 0) {
        this.setFunnelMotorVoltage(funnelMotorPower.get());
      }
      else if (funnelMotorCurrent.get() != 0) {
        this.setFunnelMotorCurrent(funnelMotorCurrent.get());
      }
      else if (funnelMotorPosition.get() != 0) {
        this.setFunnelMotorPosition(funnelMotorPosition.get());
      }
    } else {
      runStateMachine();
    }
  }

  // when testing, set the INDEXER motor power, current, or position based on the Tunables (if non-zero)
if (testingMode.get() != 0) {
      if (indexerMotorPower.get() != 0) {
        this.setIndexerMotorVoltage(indexerMotorPower.get());
      }
      else if (indexerMotorCurrent.get() != 0) {
        this.setIndexerMotorCurrent(indexerMotorCurrent.get());
      }
      else if (indexerMotorPosition.get() != 0) {
        this.setIndexerMotorPosition(indexerMotorPosition.get());
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
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   */
  public void setFunnelMotorPosition(double position) {
    io.setFunnelMotorPosition(position, POSITION_FEEDFORWARD);
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

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   */
  public void setIndexerMotorPosition(double position) {
    io.setIndexerMotorPosition(position, POSITION_FEEDFORWARD);
  }

//Whichever line of code does something with the motors, i replaced it with 2 lines that do the same exact thing but for the funnel and indexer motor, unsure if this is correct
  private Command getSystemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> FaultReporter.getInstance().clearFaults(SUBSYSTEM_NAME)),
            Commands.run(() -> io.setFunnelMotorVoltage(3.6)).withTimeout(1.0),
            Commands.run(() -> io.setIndexerMotorVoltage(3.6)).withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.velocityRPM < 2.0) {
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
                  if (inputs.velocityRPM > -2.0) {
                    FaultReporter.getInstance()
                        .addFault(
                            SUBSYSTEM_NAME,
                            "[System Check] Subsystem motor moving too slow or in the wrong direction",
                            false,
                            true);
                  }
                }))
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(Commands.runOnce(() -> io.setFunnelMotorVoltage(0.0))),
        .andThen(Commands.runOnce(() -> io.setIndexerMotorVoltage(0.0)));
  }

  private void setFunnelMotorVelocity(double velocity)
  {
    io.setFunnelMotorVelocity(velocity);
  }

  private void setIndexerMotorVelocity(double velocity)
  {
    io.setIndexerMotorVelocity(velocity);
  }

// method to shoot coral which assigns coral  button presed to true
  private void shootCoral()
  {
    shootCoralButtonPressed = true;
  }

  // method to remove algae which assins the remove algae button pressed to true
  private void removeAlgae()
  {
    removeAlgaeButtonPressed = true;
  }
