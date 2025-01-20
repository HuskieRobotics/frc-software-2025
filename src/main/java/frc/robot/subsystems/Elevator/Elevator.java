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

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();


    /**
   * Create a new climber with its associated hardware interface object.
   *
   * @param io the hardware interface object for this subsystem
   */

    public Elevator(ElevatorIO io){
        this.elevatorIO = io;

        io.zeroPosition();

        //SysIdRoutineChooser.getInstance().addOption("Subsystem Voltage", sysIdRoutine);

        FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getElevatorSystemCheckCommand());

        /*
         * Add all shuffleboard tabs and widgets
         */
        registerElevatorHeightCommands();
        
    }

    // commands to be shown in QFRC dashboard for the operator (untested)
    // should these go in RobotContainer / the class we will make for subsystem commands?
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
                        goToPosition(ReefBranch.HARDSTOP);
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
                elevatorIO.setPosition(MAX_HEIGHT);
            }
        }
    

    public boolean isAtPosition(ReefBranch reefBranch){
        return Math.abs(inputs.posInches-reefBranch) < TOLERANCE; 
    }

    // TODO: Implement system check method
    public Command getElevatorSystemCheckCommand() {
        return null;
    }


    public void goToPosition(ReefBranch reefBranch){
        switch (reefBranch) {
            case L1:
                elevatorIO.setPosition(L1_HEIGHT);
                break;

            case L2:
                elevatorIO.setPosition(L2_HEIGHT);
                break;

            case L3:
                elevatorIO.setPosition(L3_HEIGHT);
                break;      
            
            case L4:
                elevatorIO.setPosition(L4_HEIGHT);
                break;

            case ALGAE_1:
                elevatorIO.setPosition(ALGAE1_HEIGHT);
                break;

            case ALGAE_2:    
                elevatorIO.setPosition(ALGAE2_HEIGHT);
                break;
            default:
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
