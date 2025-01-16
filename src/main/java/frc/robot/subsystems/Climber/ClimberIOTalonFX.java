import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.Climber.ClimberConstants;

public class ClimberIOTalonFX implements ClimberIO {
  private TalonFX climberMotor;

  public ClimberIOTalonFX() {
    configClimberMotors();
  }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // Update loggable values here (using status signals)
    }

  private void configClimberMotors() {
    climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_CAN_ID);
  }
}
