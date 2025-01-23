package frc.robot.subsystems.removal;



public class Removal extends SubsystemBase{
    private RemovalIO io;
    
    public Removal(RemovalIO io){
        this.io = io;
    }

    private void rollRoller() {
        io.setRollerMotorVoltage(REMOVAL_VOLTAGE_ROLLERS);
    }
}
