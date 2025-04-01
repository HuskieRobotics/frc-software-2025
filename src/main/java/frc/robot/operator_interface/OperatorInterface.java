// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  // drivetrain, generic

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetPoseToVisionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getTranslationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRotationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLock180Button() {
    return new Trigger(() -> false);
  }

  public default Trigger getVisionIsEnabledTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdDynamicForward() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdDynamicReverse() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdQuasistaticForward() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdQuasistaticReverse() {
    return new Trigger(() -> false);
  }

  // DRIVER TRIGGERS
  public default Trigger getExtendCageCatcherButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getPrepToScoreButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getScoreButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getPrepAndAutoScoreCoralButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDriveToNearestCoralStationButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getInitiateClimbButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRetractClimberSlowButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getExtendClimberSlowButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getExtendClimberToMatchPositionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRaiseElevatorSlowButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLowerElevatorSlowButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getOverrideDriveToPoseButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getCurrentPoseButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getInterruptAll() {
    return new Trigger(() -> false);
  }

  public default Trigger getZeroClimberButton() {
    return new Trigger(() -> false);
  }

  // OPERATOR TRIGGERS
  public default Trigger getEnablePrimaryIRSensorsTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getEnableAutoScoringTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getLevel1Trigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getLevel2Trigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getLevel3Trigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getLevel4Trigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getReefBranchATrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getReefBranchBTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getReefBranchCTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getReefBranchDTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getReefBranchETrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getReefBranchFTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getReefBranchGTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getReefBranchHTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getReefBranchITrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getReefBranchJTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getReefBranchKTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getReefBranchLTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getAlgaeBargeTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getAlgaeProcessorTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getAlgaeDropTrigger() {
    return new Trigger(() -> false);
  }
}
