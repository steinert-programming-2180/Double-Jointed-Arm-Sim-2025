// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final int kMotorPort = 0;
  public static final int kTeleMotorPort = 1;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kTeleEncoderAChannel = 2;
  public static final int kTeleEncoderBChannel = 3;
  public static final int kJoystickPort = 0;

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "Arm_P";
  public static final String kArmIKey = "Arm_I";
  public static final String kArmDKey = "Arm_D";

  public static final String kTelePositionKey = "TelePosition";
  public static final String kTelePKey = "Tele_P";
  public static final String kTeleIKey = "Tele_I";
  public static final String kTeleDKey = "Tele_D";

  public static final double kTelescopeMin = Units.inchesToMeters(28.25);
  public static final double kTelescopeMax = Units.inchesToMeters(73.50);

  // The P gain for the PID controller that drives this arm.
  public static final double kDefaultArmKp = 50.0;
  public static final double kDefaultArmKi = 0.0;
  public static final double kDefaultArmKd = 0.0;
  public static final double kDefaultArmSetpointDegrees = 200.0;
  public static final double kArmDeadband = 1.0;

  public static final double kDefaultTeleKp = 25.0;
  public static final double kDefaultTeleKi = 0.0;
  public static final double kDefaultTeleKd = 0.0;
  public static final double kDefaultTeleSetpointMeters = kTelescopeMin;

  public static final double kArmReduction = 200;
  public static final double kTelescopeReduction = 15.60;
  public static final double kArmMass = Units.lbsToKilograms(13.715); // Kilograms
  public static final double kWristMass = Units.lbsToKilograms(10); // Kilograms
  public static final double kTelescopeDrumRadius = Units.inchesToMeters(0.5);
  public static final double kArmLength = Units.inchesToMeters(28.25);
  public static final double kMinAngleRads = Units.degreesToRadians(-75);
  public static final double kMaxAngleRads = Units.degreesToRadians(255);

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses) for arm
  //  = elevator drum radius / 4096 pulses for elevator
  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
  public static final double kTelescopeEncoderDistPerPulse = 2.0 * kTelescopeDrumRadius / 4096;
}
