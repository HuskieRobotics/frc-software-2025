package frc.robot.subsystems.intake;


import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
    private static final String SUBSYSTEM_NAME = "INTAKE";

    private final IntakeIO io;
    
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    
    private final IntakeMotor[] intakeMotors = {
        IntakeMotor.ROLLER, IntakeMotor.PIVOT
    };

    private boolean isDeployed;
    private boolean isRetracted;
    
    private boolean automationEnabled;
    private int intakeandPivotTimeout;

    private final LoggedTunableNumber rollerVelocity = new LoggedTunableNumber( "Intake/Roller Velocity", 0.0);
    private final LoggedTunableNumber pivotPosition = new LoggedTunableNumber( "Intake/Pivot Position", 0.0);

    enum IntakeMotor{
        ROLLER,
        PIVOT
    }

    public Intake(IntakeIO io){
        this.io = io;
    }


    @Override
    public void periodic(){
        io.updateInputs(inputs);
    }

    public void deploy(){

    }

    public void retract(){
        io.setPivotRotationPosition(IntakeConstants.INTAKE_POSITION_PIVOT_DEGREES);
    }

    public void extend(){
        io.setPivotRotationPosition(IntakeConstants.CARPET_POSITION_PIVOT_DEGREES);
    }

    public void intake(){
        io.setRollerMotorVelocity(IntakeConstants.INTAKE_VELOCITY_ROLLERS_RPS);
    }
        
    public void stopRoller(){
        io.setRollerMotorVelocity(0);
    }

    public void outtake(){
        io.setRollerMotorVelocity(IntakeConstants.OUTTAKE_VELOCITY_ROLLERS_RPS);
    }
}