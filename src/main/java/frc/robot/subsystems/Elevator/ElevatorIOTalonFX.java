package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team6328.util.LoggedTunableNumber;

public class ElevatorIOTalonFX implements ElevatorIO {


    private TalonFX elevatorMotorLead;
    private TalonFX elevatorMotorFollower;

    private VoltageOut elevatorMotorLeadVoltageRequest;
    private VoltageOut elevatorMotorFollowerVoltageRequest;

    MotionMagicExpoVoltage elevatorMotorLeadExpoVoltageRequest;

    private TalonFXConfiguration elevatorMotorLeadConfig;
    private TalonFXConfiguration elevatorMotorFollowerConfig;
    

    private Alert configAlert = 
        new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

    private StatusSignal<Double> elevatorLeadStatorCurrentStatusSignal;
    private StatusSignal<Double> elevatorFollowerStatorCurrentStatusSignal;

    private StatusSignal<Double> elevatorLeadSupplyCurrentStatusSignal; 
    private StatusSignal<Double> elevatorFollowerSupplyCurrentStatusSignal; 

    private StatusSignal<Double> elevatorPositionStatusSignal;
    
    private StatusSignal<Double> elevatorLeadTempStatusSignal;
    private StatusSignal<Double> elevatorFollowerTempStatusSignal;


    // Tunable constants
    private final LoggedTunableNumber kPslot0 = new LoggedTunableNumber("Elevator/kPslot0", ElevatorConstants.KP_SLOT0);
    private final LoggedTunableNumber kIslot0 = new LoggedTunableNumber("Elevator/kIslot0", ElevatorConstants.KI_SLOT0);
    private final LoggedTunableNumber kDslot0 = new LoggedTunableNumber("Elevator/kDslot0", ElevatorConstants.KD_SLOT0);
    private final LoggedTunableNumber kSslot0 = new LoggedTunableNumber("Elevator/kSslot0", ElevatorConstants.KS_SLOT0);
    private final LoggedTunableNumber kVslot0 = new LoggedTunableNumber("Elevator/kVslot0", ElevatorConstants.KV_SLOT0);
    private final LoggedTunableNumber kAslot0 = new LoggedTunableNumber("Elevator/kAslot0", ElevatorConstants.KA_SLOT0);
    private final LoggedTunableNumber kVExposlot0 = new LoggedTunableNumber("Elevator/kVExposlot0", ElevatorConstants.KV_EXPO_SLOT0);
    private final LoggedTunableNumber kAExposlot0 = new LoggedTunableNumber("Elevator/kAExposlot0", ElevatorConstants.KA_EXPO_SLOT0);
    private final LoggedTunableNumber kGslot0 = new LoggedTunableNumber("Elevator/kGslot0", ElevatorConstants.KG_SLOT0);

    private final LoggedTunableNumber kPslot1 = new LoggedTunableNumber("Elevator/kPslot1", ElevatorConstants.KP_SLOT1);
    private final LoggedTunableNumber kIslot1 = new LoggedTunableNumber("Elevator/kIslot1", ElevatorConstants.KI_SLOT1);
    private final LoggedTunableNumber kDslot1 = new LoggedTunableNumber("Elevator/kDslot1", ElevatorConstants.KD_SLOT1);
    private final LoggedTunableNumber kSslot1 = new LoggedTunableNumber("Elevator/kSslot1", ElevatorConstants.KS_SLOT1);
    private final LoggedTunableNumber kVslot1 = new LoggedTunableNumber("Elevator/kVslot1", ElevatorConstants.KV_SLOT1);
    private final LoggedTunableNumber kAslot1 = new LoggedTunableNumber("Elevator/kAslot1", ElevatorConstants.KA_SLOT1);
    private final LoggedTunableNumber kVExposlot1 = new LoggedTunableNumber("Elevator/kVExposlot1", ElevatorConstants.KV_EXPO_SLOT1);
    private final LoggedTunableNumber kAExposlot1 = new LoggedTunableNumber("Elevator/kAExposlot1", ElevatorConstants.KA_EXPO_SLOT1);
    private final LoggedTunableNumber kGslot1 = new LoggedTunableNumber("Elevator/kGslot1", ElevatorConstants.KG_SLOT1);

    private final LoggedTunableNumber kPslot2 = new LoggedTunableNumber("Elevator/kPslot2", ElevatorConstants.KP_SLOT2);
    private final LoggedTunableNumber kIslot2 = new LoggedTunableNumber("Elevator/kIslot2", ElevatorConstants.KI_SLOT2);
    private final LoggedTunableNumber kDslot2 = new LoggedTunableNumber("Elevator/kDslot2", ElevatorConstants.KD_SLOT2);
    private final LoggedTunableNumber kSslot2 = new LoggedTunableNumber("Elevator/kSslot2", ElevatorConstants.KS_SLOT2);
    private final LoggedTunableNumber kVslot2 = new LoggedTunableNumber("Elevator/kVslot2", ElevatorConstants.KV_SLOT2);
    private final LoggedTunableNumber kAslot2 = new LoggedTunableNumber("Elevator/kAslot2", ElevatorConstants.KA_SLOT2);
    private final LoggedTunableNumber kVExposlot2 = new LoggedTunableNumber("Elevator/kVExposlot2", ElevatorConstants.KV_EXPO_SLOT2);
    private final LoggedTunableNumber kAExposlot2 = new LoggedTunableNumber("Elevator/kAExposlot2", ElevatorConstants.KA_EXPO_SLOT2);
    private final LoggedTunableNumber kGslot2 = new LoggedTunableNumber("Elevator/kGslot2", ElevatorConstants.KG_SLOT2);



    private final LoggedTunableNumber cruiseVelocity = new LoggedTunableNumber("Elevator/Cruise Velocity", 0);
    private final LoggedTunableNumber acceleration = new LoggedTunableNumber("Elevator/Acceleration", 0);
    private final LoggedTunableNumber jerk = new LoggedTunableNumber("Elevator/Jerk", 0);

    private final LoggedTunableNumber voltageSuppliedLoggedTunableNumber = 
        new LoggedTunableNumber("Elevator/Voltage Supplied", 0);
    private final LoggedTunableNumber statorCurrentAmpsLoggedTunableNumber = 
        new LoggedTunableNumber("Elevator/Stator Current Amps", 0);
    private final LoggedTunableNumber supplyCurrentAmpsLoggedTunableNumber = 
        new LoggedTunableNumber("Elevator/Supply Current Amps", 0);
    private final LoggedTunableNumber closedLoopErrorLoggedTunableNumber = 
        new LoggedTunableNumber("Elevator/Closed Loop Error", 0);
    private final LoggedTunableNumber closedLoopReferenceLoggedTunableNumber = 
        new LoggedTunableNumber("Elevator/Closed Loop Reference", 0);
    private final LoggedTunableNumber positionInchesLoggedTunableNumber = 
        new LoggedTunableNumber("Elevator/Position Inches", 0);

    
    
    public ElevatorIOTalonFX() {

        elevatorMotorLead = new TalonFX(ElevatorConstants.LEAD_MOTOR_ID);
        elevatorMotorFollower = new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_ID);

        elevatorLeadStatorCurrentStatusSignal = elevatorMotorLead.getStatorCurrent();
        elevatorFollowerStatorCurrentStatusSignal = elevatorMotorFollower.getStatorCurrent();

        elevatorLeadSupplyCurrentStatusSignal = elevatorMotorLead.getSupplyCurrent();
        elevatorFollowerSupplyCurrentStatusSignal = elevatorMotorFollower.getSupplyCurrent();

        elevatorPositionStatusSignal = elevatorMotorLead.getPosition();

        elevatorLeadTempStatusSignal = elevatorMotorLead.getDeviceTemp();
        elevatorFollowerTempStatusSignal = elevatorMotorFollower.getDeviceTemp();

        elevatorMotorLeadVoltageRequest = new VoltageOut(0);
        elevatorMotorFollowerVoltageRequest = new VoltageOut(0);
    }

    private void configElevatorMotorLead(TalonFX elevevatorMotorLead){

        elevatorMotorLeadConfig = new TalonFXConfiguration();

        MotionMagicConfigs leadMotorConfig  = elevatorMotorLeadConfig.MotionMagic;

        /*
         * Add current limits here?
        */

        elevatorMotorLeadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        elevatorMotorLeadConfig.Slot0.kP = kPslot0.get();
        elevatorMotorLeadConfig.Slot0.kI = kIslot0.get();
        elevatorMotorLeadConfig.Slot0.kD = kDslot0.get();
        elevatorMotorLeadConfig.Slot0.kS = kSslot0.get();
        elevatorMotorLeadConfig.Slot0.kV = kVslot0.get();
        elevatorMotorLeadConfig.Slot0.kA = kAslot0.get();
        elevatorMotorLeadConfig.Slot0.kG = kGslot0.get();

        elevatorMotorLeadConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static);

        elevatorMotorLeadConfig.Slot1.kP = kPslot1.get();
        elevatorMotorLeadConfig.Slot1.kI = kIslot1.get();
        elevatorMotorLeadConfig.Slot1.kD = kDslot1.get();
        elevatorMotorLeadConfig.Slot1.kS = kSslot1.get();
        elevatorMotorLeadConfig.Slot1.kV = kVslot1.get();
        elevatorMotorLeadConfig.Slot1.kA = kAslot1.get();
        elevatorMotorLeadConfig.Slot1.kG = kGslot1.get();

        elevatorMotorLeadConfig.Slot1.withGravityType(GravityTypeValue.Elevator_Static);

        elevatorMotorLeadConfig.Slot2.kP = kPslot2.get();
        elevatorMotorLeadConfig.Slot2.kI = kIslot2.get();
        elevatorMotorLeadConfig.Slot2.kD = kDslot2.get();
        elevatorMotorLeadConfig.Slot2.kS = kSslot2.get();
        elevatorMotorLeadConfig.Slot2.kV = kVslot2.get();
        elevatorMotorLeadConfig.Slot2.kA = kAslot2.get();
        elevatorMotorLeadConfig.Slot2.kG = kGslot2.get();

        elevatorMotorLeadConfig.Slot2.withGravityType(GravityTypeValue.Elevator_Static);

        leadMotorConfig.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY;
        leadMotorConfig.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;
        leadMotorConfig.MotionMagicJerk = ElevatorConstants.JERK;

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = elevatorMotorLead.getConfigurator().apply(elevatorMotorLeadConfig);
            if (status.isOK()) {
                break;
            }
        }
        
        if (!status.isOK()) {
            configAlert.set(true);
            configAlert.setText(status.toString());
        }
        
        FaultReporter.getInstance().registerHardware(ElevatorConstants.SUBSYSTEM_NAME, "Elevator Motor Lead", elevatorMotorLead);
    }

    // Update Inputs
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

        BaseStatusSignal.refreshAll(
            elevatorPositionStatusSignal,

            elevatorLeadStatorCurrentStatusSignal,
            elevatorFollowerStatorCurrentStatusSignal,

            elevatorLeadSupplyCurrentStatusSignal,
            elevatorFollowerSupplyCurrentStatusSignal,

            elevatorLeadTempStatusSignal,
            elevatorFollowerTempStatusSignal
        );

        inputs.voltageSuppliedLead = voltageSuppliedLoggedTunableNumber.get();
        inputs.voltageSuppliedFollower = voltageSuppliedLoggedTunableNumber.get();  

        inputs.statorCurrentAmpsLead = statorCurrentAmpsLoggedTunableNumber.get();
        inputs.statorCurrentAmpsFollower = statorCurrentAmpsLoggedTunableNumber.get();

        inputs.supplyCurrentAmpsLead = supplyCurrentAmpsLoggedTunableNumber.get();
        inputs.supplyCurrentAmpsFollower = supplyCurrentAmpsLoggedTunableNumber.get();

        inputs.closedLoopError = closedLoopErrorLoggedTunableNumber.get();
        inputs.closedLoopReference = closedLoopReferenceLoggedTunableNumber.get();

        inputs.positionInches = positionInchesLoggedTunableNumber.get();
    }

    // Set motor voltage
    public void setMotorVoltage(double voltage) {
        elevatorMotorFollower.setControl(elevatorMotorFollowerVoltageRequest.withOutput(voltage));
        elevatorMotorLead.setControl(elevatorMotorLeadVoltageRequest.withOutput(-voltage));
        
    }

    // Add softlimits to elevator to prevent hitting hardstops

    public void setElevatorSoftLimits(){



    }
}


