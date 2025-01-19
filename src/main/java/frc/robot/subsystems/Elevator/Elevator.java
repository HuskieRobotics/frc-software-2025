package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3015.subsystem.selfcheck.SelfChecking;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.subsystems.Elevator.ElevatorConstants.ReefBranch;
import frc.robot.subsystems.Elevator.ElevatorIO.ElevatorIOInputs;
import java.lang.Math;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase{
    

    private ElevatorIO elevatorIO;

    static final String SUBSYSTEM_NAME = "ELEVATOR";

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged(); // Is this an auto generated class?


    /**
   * Create a new climber with its associated hardware interface object.
   *
   * @param io the hardware interface object for this subsystem
   */

    public Elevator(ElevatorIO io){
        this.elevatorIO = io;

        io.setPositionToZero();

        //SysIdRoutineChooser.getInstance().addOption("Subsystem Voltage", sysIdRoutine);

        FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());

        /*
         * Add all shuffleboard tabs and widgets
         */
        registerElevatorHeightCommands();
        
    }

    // commands to be shown in qfrc dashboard for the operator (untested)
    // TODO: see if these commands show up in qfrc dashboard

    private void registerElevatorHeightCommands() {
        registerGroundCommand(); 
        registerL1Command();
        registerL2Command();
        registerL3Command();
        registerL4Command();
    }

    private void registerGroundCommand() {
        SmartDashboard.putData(
            "Ground",
            Commands.runOnce(
                    () -> {
                        goToPosition(ReefBranch.Hardstop);
                    })
                .ignoringDisable(true)
                .withName("Ground"));
    }

    private void registerL1Command() {
        SmartDashboard.putData(
            "L1",
            Commands.runOnce(
                    () -> {
                        goToPosition(ReefBranch.L1);
                    })
                .ignoringDisable(true)
                .withName("L1"));
    }

    private void registerL2Command() {
        SmartDashboard.putData(
            "L2",
            Commands.runOnce(
                    () -> {
                        goToPosition(ReefBranch.L1);
                    })
                .ignoringDisable(true)
                .withName("L2"));
    }

    private void registerL3Command() {
        SmartDashboard.putData(
            "L3",
            Commands.runOnce(
                    () -> {
                        goToPosition(ReefBranch.L1);
                    })
                .ignoringDisable(true)
                .withName("L3"));
    }

    private void registerL4Command() {
        SmartDashboard.putData(
            "L4",
            Commands.runOnce(
                    () -> {
                        goToPosition(ReefBranch.L1);
                    })
                .ignoringDisable(true)
                .withName("L4"));
    }

    /*
     * Implement periodic method for Elevator
     */

     @Override
        public void periodic(){
            elevatorIO.updateInputs(inputs);
            Logger.processInputs(SUBSYSTEM_NAME, inputs);


            if(inputs.posInches > MAX_HEIGHT){
                elevatorIO.setMotorPosition(MAX_HEIGHT);
            }
        }
    

    public boolean isAtPosition(ReefBranch reefBranch){
        return Math.abs(inputs.posInches-reefBranch) < TOLERANCE; 
    }

    public void goToPosition(ReefBranch reefBranch){
        switch (reefBranch) {
            case L1:
                elevatorIO.setMotorPosition(L1_HEIGHT);
                break;

            case L2:
                elevatorIO.setMotorPosition(L2_HEIGHT);
                break;

            case L3:
                elevatorIO.setMotorPosition(L3_HEIGHT);
                break;      
            
            case L4:
                elevatorIO.setMotorPosition(L4_HEIGHT);
                break;

            case Algae1:
                elevatorIO.setMotorPosition(ALGAE1_HEIGHT);
                break;

            case Algae2:    
                elevatorIO.setMotorPosition(ALGAE2_HEIGHT);
                break;
        }
    }

    public double getPosition(){

        return inputs.posInches;
    }


    public double rotationsToHeight(double rotations){
        return rotations * CONVERSION_FACTOR; // FIXME: Fix this value
    }
    




}
