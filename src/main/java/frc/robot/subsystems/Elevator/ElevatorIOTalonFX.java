package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team6328.util.LoggedTunableNumber;

public class ElevatorIOTalonFX implements ElevatorIO {


    private TalonFX elevatorMotorRight;
    private TalonFX elevatorMotorLeft;

    private VoltageOut elevatorMotorRightVelocityRequest;
    private VoltageOut elevatorMotorLeftVelocityRequest;


    private TalonFXConfiguration elevatorMotorRightConfig;
    

    private Alert configAlert = 
        new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

    private StatusSignal<Double> elevatorStatorCurrentStatusSignal;
    private StatusSignal<Double> elevatorSupplyCurrentStatusSignal;
    private StatusSignal<Double> elevatorPosStatusSignal;
    private StatusSignal<Double> elevatorTempStatusSignal;


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

    private void configElevatorMotorLeft(TalonFX elevevatorMotorLeft){
        TalonFXConfiguration elevatorMotorLeftConfig;

        elevatorMotorLeftConfig = new TalonFXConfiguration();

        MotionMagicConfigs leftMotorConfig  = elevatorMotorLeftConfig.MotionMagic;

        /*
         * Add current limits here?
        */

        elevatorMotorLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        elevatorMotorLeftConfig.Slot0.kP = kPslot0.get();
        elevatorMotorLeftConfig.Slot0.kI = kIslot0.get();
        elevatorMotorLeftConfig.Slot0.kD = kDslot0.get();
        elevatorMotorLeftConfig.Slot0.kS = kSslot0.get();
        elevatorMotorLeftConfig.Slot0.kV = kVslot0.get();
        elevatorMotorLeftConfig.Slot0.kA = kAslot0.get();
        elevatorMotorLeftConfig.Slot0.kG = kGslot0.get();

        elevatorMotorLeftConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static);

        leftMotorConfig.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY;
        leftMotorConfig.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;
        leftMotorConfig.MotionMagicJerk = ElevatorConstants.JERK;

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = elevatorMotorLeft.getConfigurator().apply(elevatorMotorLeftConfig);
            if (status.isOK()) {
                break;
            }
        }
        
        if (!status.isOK()) {
            configAlert.set(true);
            configAlert.setText(status.toString());
        }
        
        FaultReporter.getInstance().registerHardware(Elevator.SUBSYSTEM_NAME, "Elevator Motor Left", elevatorMotorLeft);
    }
        
    




    public ElevatorIOTalonFX() {
        
        elevatorMotorLeftVelocityRequest = new VoltageOut(0);
        elevatorMotorRightVelocityRequest = new VoltageOut(0);
        

    }

    // Update Inputs
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.voltageSupplied = voltageSuppLoggedTunableNumber.get();
        inputs.statorCurrentAmps = statorCurrentAmpsLoggedTunableNumber.get();
        inputs.supplyCurrentAmps = supplyCurrentAmpsLoggedTunableNumber.get();
        inputs.closedLoopError = closedLoopErrorLoggedTunableNumber.get();
        inputs.closedLoopReference = closedLoopReferenceLoggedTunableNumber.get();
        inputs.posInches = posInchesLoggedTunableNumber.get();

        BaseStatusSignal.refreshAll(
            elevatorPosStatusSignal,
            elevatorSupplyCurrentStatusSignal,
            elevatorPosStatusSignal,
            elevatorTempStatusSignal
        );
    }

    // Set motor voltage
    public void setMotorVoltage(double voltage) {
        elevatorMotorRight.setControl(elevatorMotorRightVelocityRequest.withOutput(voltage));
        elevatorMotorLeft.setControl(elevatorMotorLeftVelocityRequest.withOutput(-voltage));
        
    }
}


