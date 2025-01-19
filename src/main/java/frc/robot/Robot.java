// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Arm;

/** This is a sample program to demonstrate the use of arm simulation with existing code. */
public class Robot extends TimedRobot {
  private final Arm m_arm = new Arm();
  // private final Joystick m_joystick = new Joystick(Constants.kJoystickPort);
  private final PS5Controller m_joystick = new PS5Controller(0);

  public Robot() {}

  @Override
  public void simulationPeriodic() {
    m_arm.simulationPeriodic();
  }

  @Override
  public void teleopInit() {
    m_arm.loadPreferences();
  }

  @Override
  public void teleopPeriodic() {
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
    
    if (m_joystick.getL1Button()) {
      m_arm.retractArm();
    } else if (m_joystick.getR1Button()) {
      m_arm.extendArm();
    } else {
      // Otherwise, we disable the motor.
      m_arm.stopTelescope();
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
