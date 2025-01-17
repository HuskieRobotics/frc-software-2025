package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.LoggedTunableNumber;

public class ElevatorIOTalonFX {

    private Alert configAlert = 
        new Alert("Failed to apply configuration for subsystem.", AlertType.ERROR);

    private StatusSignal<Double> elevatorStatorCurrentStatusSignal;
    private StatusSignal<Double> elevatorSupplyCurrentStatusSignal;
    private StatusSignal<Double> elevatorPosStatusSignal;

    private TalonFX elevatorMotor;

    // Tunable constants
    private final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", ElevatorConstants.KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", ElevatorConstants.KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", ElevatorConstants.KD);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", ElevatorConstants.KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", ElevatorConstants.KV);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", ElevatorConstants.KA);
    private final LoggedTunableNumber kVExpo = new LoggedTunableNumber("Elevator/kVExpo", ElevatorConstants.KV_EXPO);
    private final LoggedTunableNumber kAExpo = new LoggedTunableNumber("Elevator/kAExpo", ElevatorConstants.KA_EXPO);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", ElevatorConstants.KG);

    private final LoggedTunableNumber cruiseVelocity = new LoggedTunableNumber("Elevator/Cruise Velocity", 0);
    private final LoggedTunableNumber acceleration = new LoggedTunableNumber("Elevator/Acceleration", 0);
    private final LoggedTunableNumber jerk = new LoggedTunableNumber("Elevator/Jerk", 0);

    private final LoggedTunableNumber voltageSuppLoggedTunableNumber = 
        new LoggedTunableNumber("Elevator/Voltage Supplied", 0);
    private final LoggedTunableNumber statorCurrentAmpsLoggedTunableNumber = 
        new LoggedTunableNumber("Elevator/Stator Current Amps", 0);
    private final LoggedTunableNumber supplyCurrentAmpsLoggedTunableNumber = 
        new LoggedTunableNumber("Elevator/Supply Current Amps", 0);
    private final LoggedTunableNumber closedLoopErrorLoggedTunableNumber = 
        new LoggedTunableNumber("Elevator/Closed Loop Error", 0);
    private final LoggedTunableNumber closedLoopReferenceLoggedTunableNumber = 
        new LoggedTunableNumber("Elevator/Closed Loop Reference", 0);
    private final LoggedTunableNumber posInchesLoggedTunableNumber = 
        new LoggedTunableNumber("Elevator/Position Inches", 0);

    // Constructor
    public ElevatorIOTalonFX() {
        // Constructor logic here
    }

    // Update Inputs
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.voltageSupplied = voltageSuppLoggedTunableNumber.get();
        inputs.statorCurrentAmps = statorCurrentAmpsLoggedTunableNumber.get();
        inputs.supplyCurrentAmps = supplyCurrentAmpsLoggedTunableNumber.get();
        inputs.closedLoopError = closedLoopErrorLoggedTunableNumber.get();
        inputs.closedLoopReference = closedLoopReferenceLoggedTunableNumber.get();
        inputs.posInches = posInchesLoggedTunableNumber.get();
    }

    // Set motor voltage
    public void setMotorVoltage(double voltage) {
        elevatorMotor.setControl(new VoltageOut(voltage));
    }
}
