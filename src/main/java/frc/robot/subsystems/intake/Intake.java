package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
  private static final String SUBSYSTEM_NAME = "INTAKE";

  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final IntakeMotor[] intakeMotors = {IntakeMotor.ROLLER, IntakeMotor.PIVOT};

  private boolean isDeployed = false;
  private boolean isRetracted = false;

  private boolean automationEnabled;
  private int intakeandPivotTimeout;

  private final LoggedTunableNumber rollerVelocity =
      new LoggedTunableNumber("Intake/Roller Velocity", 0.0);
  private final LoggedTunableNumber pivotPosition =
      new LoggedTunableNumber("Intake/Pivot Position", 0.0);

  enum IntakeMotor {
    ROLLER,
    PIVOT
  }

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  private void deploy() {
    if (isDeployed == true) {
      this.intake();
    } else {
      this.stopRoller();
    }
  }

  private void shootOut() {
    if (isRetracted == true) {
      this.outtake();
    } else {
      this.stopRoller();
    }
  }

  public void retract() {
    io.setPivotRotationPosition(IntakeConstants.INTAKE_POSITION_PIVOT_DEGREES);
    isDeployed = false;
  }

  public void extend() {
    io.setPivotRotationPosition(IntakeConstants.CARPET_POSITION_PIVOT_DEGREES);
    isDeployed = true;
  }

  public void intake() {
    io.setRollerMotorVelocity(IntakeConstants.INTAKE_VELOCITY_ROLLERS_RPS);
  }

  public void stopRoller() {
    io.setRollerMotorVelocity(0);
  }

  public void outtake() {
    io.setRollerMotorVelocity(IntakeConstants.OUTTAKE_VELOCITY_ROLLERS_RPS);
  }
}
