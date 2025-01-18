import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.Climber.ClimberConstants;

public class ClimberIOTalonFX implements ClimberIO {
  private TalonFX climberMotor;

  private VoltageOut climberVoltageRequest;

  private StatusSignal<Double> voltage;
  private StatusSignal<Double> staterCurrentAmps;
  private StatusSignal<Double> supplyCurrentAmps;
  private StatusSignal<Double> tempCelcius;
  private StatusSignal<Double> position_units_per_rotation;


  public ClimberIOTalonFX() {
    configClimberMotors();

    climberVoltageRequest = new VoltageOut(0);
    
    

  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
      // Update loggable values here (using status signals)
      BaseStatusSignal.refreshAll(
        voltage,
        staterCurrentAmps,
        supplyCurrentAmps,
        tempCelcius,
        position_units_per_rotation
      );

      inputs.voltage = voltage.getValueAsDouble();
      inputs.staterCurrentAmps = staterCurrentAmps.getValueAsDouble();
      inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
      inputs.tempCelcius = tempCelcius.getValueAsDouble();
      inputs.position_units_per_rotation = position_units_per_rotation.getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    climberMotor.setControl(climberVoltageRequest.withOutput(voltage));
  }

  public void zeroPosition() {
    climberMotor.setPosition(0, 0);
  }

  private void configClimberMotors() {
    climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_CAN_ID);
    
  
  }
}
