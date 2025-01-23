package frc.robot.subsystems.removal;

import org.littletonrobotics.junction.Logger;


public class Removal extends SubsystemBase {
    private RemovalIO io;

    private final RemovalIOInputsAutoLogged removalInputs = new RemovalIOInputsAutoLogged();
    
    public Removal(RemovalIO io){
        this.io = io;
    }

    private void rollRoller() {
        io.setRollerMotorVoltage(RemovalConstants.REMOVAL_VOLTAGE_ROLLERS);
    }
}

@Override
public void periodic() {
    io.updateInputs(inputs);
}