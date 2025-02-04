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

    //getPosition() arguments in io.setVoltage are placeholders
    
    public void periodic() {
        io.updateInputs(inputs);
        if (testingMode.get() == 1) {
            //chekc voltage tunable DO NOT FORGET
        }
        else if (testingMode.get() != 0) {
            io.setVoltage(getPosition());
        }
    }

    public void extend() {
        io.setVoltage(getPosition());
    }

    public void retract() {
        io.setVoltage(getPosition());
    }

    public void reset() {
        io.setVoltage(getPosition());
    }

    public void zero() {
        io.zeroPosition();
    }

    public double getPosition() {
        return inputs.positionInches;
    }

}
