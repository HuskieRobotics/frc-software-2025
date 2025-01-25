package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import javax.swing.text.Position;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3015.subsystem.selfcheck.SelfChecking;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.subsystems.Elevator.ElevatorConstants.ReefBranch;
import frc.robot.subsystems.Elevator.ElevatorIO.ElevatorIOInputs;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

public class Elevator extends SubsystemBase{
    

    private ElevatorIO elevatorIO;

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Elevator/TestingMode", 0);

    private final LoggedTunableNumber elevatorVoltage = new LoggedTunableNumber("Elevator/Voltage", 0);

    private final LoggedTunableNumber elevatorHeight_Inches = new LoggedTunableNumber("Elevator/Height(Inches)", 0);

    private final LoggedTunableNumber rampRate = new LoggedTunableNumber("Elevator/RampRate", 0);

    private final LoggedTunableNumber stepVoltage = new LoggedTunableNumber("Elevator/StepVoltage", 0);


    public Elevator(ElevatorIO io){

        this.elevatorIO = io;

        io.zeroPosition();

        SysIdRoutineChooser.getInstance().addOption("Subsystem Voltage", sysIdRoutineStage1);
        SysIdRoutineChooser.getInstance().addOption("Subsystem Voltage", sysIdRoutineStage2);
        SysIdRoutineChooser.getInstance().addOption("Subsystem Voltage", sysIdRoutineStage3);

        FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getElevatorSystemCheckCommand());

        /*
         * Add all shuffleboard tabs and widgets
         */
        registerElevatorHeightCommands();   
      
        FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getElevatorSystemCheckCommand());
        
    }

    private final SysIdRoutine sysIdRoutineStage1 =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              null, // Use default step voltage (7 V)
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> elevatorIO.setMotorVoltage(output.in(Volts)), null, this));

    private final SysIdRoutine sysIdRoutineStage2 =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                null, // Use default step voltage (7 V)
                null, // Use default timeout (10 s)
                // Log state with SignalLogger class
                state -> SignalLogger.writeString("SysId_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> elevatorIO.setMotorVoltage(output.in(Volts)), null, this));


    private final SysIdRoutine sysIdRoutineStage3 =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                null, // Use default step voltage (7 V)
                null, // Use default timeout (10 s)
                // Log state with SignalLogger class
                state -> SignalLogger.writeString("SysId_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> elevatorIO.setMotorVoltage(output.in(Volts)), null, this));

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

            if(testingMode.get() == 1){

                if(elevatorVoltage.get() == 0){
                    elevatorIO.setVoltage(elevatorVoltage.get());
                }
                else if(elevatorHeight_Inches.get() == 0){
                    elevatorIO.setPosition(Inches.of(elevatorHeight_Inches.get()));
                }
            }
        }

    public Distance reefBranchToDistance(ReefBranch reefBranch){
        switch (reefBranch) {
            case L1:
                return L1_HEIGHT;
                break;

            case L2:
                return L2_HEIGHT;
                break;

            case L3:
                return L3_HEIGHT;
                break;      
            
            case L4:
                return L4_HEIGHT;
                break;

            case ALGAE_1:
                return ALGAE1_HEIGHT;
                break;

            case ALGAE_2:    
                return ALGAE2_HEIGHT;
                break;
            default:
                return MIN_HEIGHT;
                break;
        }

    }

    public boolean isAtPosition(ReefBranch reefBranch){
        return getPosition().equals(reefBranchToDistance(reefBranch));
    }

    // TODO: Implement system check method
    public Command getElevatorSystemCheckCommand() {
        return null;
    }


    public void goToPosition(ReefBranch reefBranch){
        elevatorIO.setPosition(reefBranchToDistance(reefBranch));
    }

    public Distance getPosition(){

        return inputs.posInches;
    }


    private double rotationsToHeight(double rotations){
        return rotations * CONVERSION_FACTOR; // FIXME: Fix this value
    }
}
