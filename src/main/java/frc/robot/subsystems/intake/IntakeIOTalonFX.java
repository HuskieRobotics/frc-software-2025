package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team3061.sim.VelocitySystemSim;

public class IntakeIOTalonFX implements IntakeIO {
    private TalonFX pivotMotor;
    private TalonFX rollerMotor;

    private VelocityTorqueCurrentFOC rollerVelocityRequest;
    private VoltageOut pivotVoltageRequest;
// roller status signals
    private StatusSignal<Double> rollerMotorVoltageStatusSignal;
    private StatusSignal<Double> rollerMotorVelocityStatusSignal;
    private StatusSignal<Double> rollerStatorCurrentStatusSignal;
    private StatusSignal<Double> rollerSupplyCurrentStatusSignal;
    private StatusSignal<Double> rollerTempCelciusStatusSignal;
    private StatusSignal<Double> rollerClosedLoopErrorStatusSignal;
    private StatusSignal<Double> rollerReferenceVelocityStatusSignal;

// pivot status signals
    private StatusSignal<Double> pivotMotorVoltageStatusSignal;
    private StatusSignal<Double> pivotPositionStatusSignal;
    private StatusSignal<Double> pivotStatorCurrentStatusSignal;
    private StatusSignal<Double> pivotTempCelciusStatusSignal;
    private StatusSignal<Double> pivotClosedLoopErrorStatusSignal;
    private StatusSignal<Double> pivotReferencePositionStatusSignal;
    private StatusSignal<Double> pivotSupplyStatusSignal;
    private StatusSignal<Double> pivotPositionDegStatusSignal;

    private VelocitySystemSim rollerSim;
    private VelocitySystemSim pivotSim;

    

    @Override
    public void updateInputs(IntakeIOINputs inputs) {
    }

    @Override
    public void setPivotRotationPosition(double position) {
    }

    @Override
    public void setPivotVoltage(double voltage) {
    }

    @Override
    public void setRollerMotorVoltage(double voltage) {
    }

    @Override
    public void setRollerMotorVelocity(double velocity) {
    }
}

