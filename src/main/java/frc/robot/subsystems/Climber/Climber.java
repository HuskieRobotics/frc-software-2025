package frc.robot.subsystems.Climber;
import frc.lib.team6328.util.LoggedTunableNumber;

public class Climber {
    private ClimberIO io;
    private ClimberState climberState;

    private enum ClimberState {
        IDLE, EXTENDING, RETRACTING, RETRACTINGSLOW
    }

    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Climber/TestingMode", 0);
    private final LoggedTunableNumber climberVoltage = new LoggedTunableNumber("Climber/Voltage", 0.0);
    
    public Climber(ClimberIO io) {
        this.io = io;
        //no state machine 
        //so then no need for all of the below? (with the climberState etc)

        this.climberState = climberState.IDLE;
    }

    public void periodic() {
        if (testingMode.get() == 0) {
            io.updateInputs(inputs);
        }
    }

    public void extend() {
        climberState = ClimberState.EXTENDING;
    }

    public void retract() {
        climberState = ClimberState.RETRACTING;
    }

    public void reset() {
        climberState = ClimberState.RETRACTINGSLOW;
    }

    public void zero() {
        this.climberState = ClimberState.IDLE;
        io.zeroPosition();
    }

    public double getPosition() {
        return 0.0; //placeholder
    }

}
