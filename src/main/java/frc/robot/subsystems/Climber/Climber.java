package frc.robot.subsystems.Climber;
import frc.lib.team6328.util.LoggedTunableNumber;

public class Climber {
    private ClimberIO io;

    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Climber/TestingMode", 0);
    private final LoggedTunableNumber climberVoltage = new LoggedTunableNumber("Climber/Voltage", 0.0);
    
    public Climber(ClimberIO io) {
        this.io = io;

    }

    //getPosition() argument in io.setVoltage() is placeholder
    
    public void periodic() {
        io.updateInputs(inputs);
        if (testingMode.get() == 1) {
            //not sure about this at all, ask how to check voltage Tunable
            climberVoltage.initDefault(getPosition());
        }
        else if (testingMode.get() != 0) {
            io.setVoltage(getPosition());
        }
    }

    public void extend() {
        io.setVoltage(ClimberConstants.EXTEND_VOLTAGE);
    }

    public void retract() {
        io.setVoltage(ClimberConstants.RETRACT_VOLTAGE);
    }

    public void reset() {
        io.setVoltage(ClimberConstants.RESET_VOLTAGE);
    }

    public void zero() {
        io.zeroPosition();
    }

    public double getPosition() {
        return inputs.positionInches;
    }

}
