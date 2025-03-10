// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two joysticks. */
public class DualJoysticksOI extends OperatorDashboard {
  private final CommandJoystick translateJoystick;
  private final CommandJoystick rotateJoystick;
  private final Trigger[] translateJoystickButtons;
  private final Trigger[] rotateJoystickButtons;

  public DualJoysticksOI(int translatePort, int rotatePort) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new Trigger[13];
    this.rotateJoystickButtons = new Trigger[13];

    for (int i = 1; i < translateJoystickButtons.length; i++) {
      translateJoystickButtons[i] = translateJoystick.button(i);
      rotateJoystickButtons[i] = rotateJoystick.button(i);
    }
  }

  // Translate Joystick
  @Override
  public double getTranslateX() {
    return -translateJoystick.getY();
  }

  @Override
  public double getTranslateY() {
    return -translateJoystick.getX();
  }

  // Translation Joystick

  @Override
  public Trigger getRaiseElevatorSlowButton() {
    return translateJoystickButtons[1];
  }

  @Override
  public Trigger getPrepToScoreCoralLeftButton() {
    return translateJoystickButtons[2];
  }

  @Override
  public Trigger getExtendClimberButton() {
    return translateJoystickButtons[3];
  }

  @Override
  public Trigger getExtendCageCatcherButton() {
    return translateJoystickButtons[4];
  }

  @Override
  public Trigger getInterruptAll() {
    return translateJoystickButtons[5];
  }

  @Override
  public Trigger getDescoreAlgaeAfterAutoButton() {
    return translateJoystickButtons[6];
  }

  @Override
  public Trigger getDriveToBargeButton() {
    return translateJoystickButtons[7];
  }

  @Override
  public Trigger getResetGyroButton() {
    return translateJoystickButtons[8];
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return translateJoystickButtons[9];
  }

  @Override
  public Trigger getLowerElevatorSlowButton() {
    return translateJoystickButtons[11];
  }

  // Rotate Joystick
  @Override
  public double getRotate() {
    return -rotateJoystick.getX();
  }

  // Rotation Joystick
  @Override
  public Trigger getScoreCoralButton() {
    return rotateJoystickButtons[1];
  }

  @Override
  public Trigger getPrepToScoreCoralRightButton() {
    return rotateJoystickButtons[2];
  }

  @Override
  public Trigger getInitiateClimbButton() {
    return rotateJoystickButtons[3];
  }

  @Override
  public Trigger getDriveToPoseOverrideButton() {
    return rotateJoystickButtons[4];
  }

  @Override
  public Trigger getResetPoseToVisionButton() {
    return rotateJoystickButtons[5];
  }

  @Override
  public Trigger getCurrentPoseButton() {
    return rotateJoystickButtons[6];
  }

  @Override
  public Trigger getRetractClimberSlowButton() {
    return rotateJoystickButtons[10];
  }

  @Override
  public Trigger getZeroClimberButton() {
    return rotateJoystickButtons[11];
  }
}
