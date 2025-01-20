// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;

/** This is a sample program to demonstrate the use of arm simulation with existing code. */
public class Robot extends TimedRobot {
  private final Arm m_arm = new Arm();
  // private final Joystick m_joystick = new Joystick(Constants.kJoystickPort);
  private final PS5Controller m_joystick = new PS5Controller(0);

  SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();


  public Robot() {}

  @Override
  public void simulationPeriodic() {
    m_arm.simulationPeriodic();
  }

  @Override
  public void robotInit() {
    m_arm.loadPreferences();

    presetChooser.setDefaultOption("Starting Position", 0);
    presetChooser.addOption("Reef L1", 1);
    presetChooser.addOption("Reef L2", 2);
    presetChooser.addOption("Reef L3", 3);
    presetChooser.addOption("Reef L4", 4);
    presetChooser.addOption("Human Player Station", 5);

    SmartDashboard.putData(presetChooser);
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    /* 

    if (m_joystick.getCircleButton()) {
      // Here, we run PID control like normal.
      m_arm.reachArmSetpoint();
    } else {
      // Otherwise, we disable the motor.
      m_arm.stopArm();
    }

    if (m_joystick.getTriangleButton()) {
      m_arm.reachWristSetpoint();
    } else {
      m_arm.stopWrist();
    }

    */
    
    if (m_joystick.getL1Button()) {
      m_arm.retractArm();
    } else if (m_joystick.getR1Button()) {
      m_arm.extendArm();
    } else {
      // Otherwise, we disable the motor.
      m_arm.stopTelescope();
    }

    switch(presetChooser.getSelected()) {
      case 0:
        m_arm.setState(Constants.armStartingPos, Constants.wristStartingPos, Constants.teleStartingPos);
        break;
      case 1:
        m_arm.setState(Constants.armL1Pos, Constants.wristL1Pos, Constants.teleStartingPos);
        break;
      case 2:
        m_arm.setState(Constants.armL2Pos, Constants.wristL2Pos, Constants.teleStartingPos);
        break;
      case 3:
        m_arm.setState(Constants.armL3Pos, Constants.wristL3Pos, Constants.teleL3Pos);
        break;
      case 4:
        m_arm.setState(Constants.armL4Pos, Constants.wristL4Pos, Constants.teleL4Pos);
        break;
      case 5:
        m_arm.setState(Constants.armStartingPos, Constants.wristStartingPos, Constants.teleStartingPos);
        break;
      default:
        m_arm.setState(Constants.armStartingPos, Constants.wristStartingPos, Constants.teleStartingPos);
        break;
    }

    
  }

  @Override
  public void close() {
    m_arm.close();
    super.close();
  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_arm.stopArm();
    m_arm.stopTelescope();
  }
}
