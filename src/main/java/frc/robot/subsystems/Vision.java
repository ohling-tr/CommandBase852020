/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Vision extends SubsystemBase {
  /**
   * Creates a new Vision.
   */
  private NetworkTable m_visionTable;
  private Servo m_camera0Servo;
  private boolean m_bIsLightOn;
  private boolean m_bIsFwdVisionOn;
  private boolean m_bIsRevVisionOn;

  private Relay m_cameraLightRelay;

  private boolean m_bIsLoadingStationAligned;
  private boolean m_bIsHighGoalAligned;

  private NetworkTableEntry m_vLoadingStationAligned;
  private NetworkTableEntry m_vHighGoalAligned;
  private NetworkTableEntry m_vFwdVisionOn;
  private NetworkTableEntry m_vRevVisionOn;
  private NetworkTableEntry m_vHighGoalOffset;
  private NetworkTableEntry m_vHighGoalDistance;

  private ShuffleboardTab m_visionTab;

  public Vision() {
    m_visionTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kVISION_TABLE_KEY);
    m_camera0Servo = new Servo(0);
    m_bIsLightOn = false;
    m_bIsFwdVisionOn = false;
    m_bIsRevVisionOn = false;

    m_vLoadingStationAligned = m_visionTable.getEntry(VisionConstants.kIS_LOADING_STATION_ALIGNED_KEY);
    m_vHighGoalAligned = m_visionTable.getEntry(VisionConstants.kIS_HIGH_GOAL_ALIGNED_KEY);
    m_vFwdVisionOn = m_visionTable.getEntry(VisionConstants.kIS_FWD_VISION_ON_KEY);
    m_vRevVisionOn = m_visionTable.getEntry(VisionConstants.kIS_REV_VISION_ON_KEY);
    m_vHighGoalOffset = m_visionTable.getEntry(VisionConstants.kVISION_OFFSET_KEY);
    m_vHighGoalDistance = m_visionTable.getEntry(VisionConstants.kVISION_DISTANCE_KEY);

    m_vLoadingStationAligned.setBoolean(false);
    m_vHighGoalAligned.setBoolean(false);
    m_vFwdVisionOn.setBoolean(false);
    m_vRevVisionOn.setBoolean(false);
    m_vHighGoalOffset.setNumber(0);
    m_vHighGoalDistance.setNumber(0);

    m_visionTab = Shuffleboard.getTab(VisionConstants.kVISION_TAB_KEY);
    m_visionTab.addNumber("Servo Angle", () -> m_camera0Servo.getAngle());
    m_visionTab.addBoolean("Is Loading Station Aligned", () -> m_bIsLoadingStationAligned);
    m_visionTab.addBoolean("Is High Goal Aligned", () -> m_bIsHighGoalAligned);

    m_bIsLoadingStationAligned = false;

    m_cameraLightRelay = new Relay(VisionConstants.kLIGHT_RELAY_PORT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_bIsLoadingStationAligned = m_vLoadingStationAligned.getBoolean(false);
    m_bIsHighGoalAligned = m_vHighGoalAligned.getBoolean(false);

    //m_vFwdVisionOn.setBoolean(m_bIsFwdVisionOn);
    //m_vRevVisionOn.setBoolean(m_bIsRevVisionOn);

    /*
    SmartDashboard.putNumber("Servo Angle", m_camera0Servo.getAngle());
    SmartDashboard.putBoolean("Is Loading Station Aligned", m_bIsLoadingStationAligned);
    SmartDashboard.putBoolean("Is High Goal Aligned", m_bIsHighGoalAligned);
    */
  }

  private void setAngle(int angle){
    m_camera0Servo.setAngle(angle);
  }

  public void setServoShooter(){
    setAngle(VisionConstants.kSERVO_SHOOTER_ANGLE);
  }

  public void setServoDown(){
    setAngle(VisionConstants.kSERVO_DOWN_ANGLE);
  }

  public void setServoUp(){
    setAngle(VisionConstants.kSERVO_UP_ANGLE);
  }

  public void setFwdVisionOn(){
    m_bIsFwdVisionOn = true;
    m_vFwdVisionOn.setBoolean(m_bIsFwdVisionOn);
  }

  public void setFwdVisionOff(){
    m_bIsFwdVisionOn = false;
    m_vFwdVisionOn.setBoolean(m_bIsFwdVisionOn);
  }

  public void setRevVisionOn(){
    m_bIsRevVisionOn = true;
    m_vRevVisionOn.setBoolean(m_bIsRevVisionOn);
  }

  public void setRevVisionOff(){
    m_bIsRevVisionOn = false;
    m_vRevVisionOn.setBoolean(m_bIsRevVisionOn);
  }

  public void setLightRelayOn(){
    m_cameraLightRelay.set(Value.kForward);
    SmartDashboard.putBoolean("VisionLight", true);
  }

  public void setLightRelayOff(){
    m_cameraLightRelay.set(Value.kOff);
    SmartDashboard.putBoolean("VisionLight", false);
  }

  public boolean isLoadingStationAligned(){
    return m_bIsLoadingStationAligned;
  }

  public boolean isHighGoalAligned(){
    return m_bIsHighGoalAligned;
  }

  public double getHighOffset(){
    return m_vHighGoalOffset.getDouble(0);

  }

  public double getHighDistance(){
    return m_vHighGoalDistance.getDouble(0);
  }

}
