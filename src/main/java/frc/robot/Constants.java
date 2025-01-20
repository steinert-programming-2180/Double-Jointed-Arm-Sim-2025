// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final int kMotorPort = 0;
  public static final int kTeleMotorPort = 1;
  public static final int kWristMotorPort = 2;

  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kTeleEncoderAChannel = 2;
  public static final int kTeleEncoderBChannel = 3;
  public static final int kWristEncoderAChannel = 4;
  public static final int kWristEncoderBChannel = 5;

  public static final int kJoystickPort = 0;

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "Arm_P";
  public static final String kArmIKey = "Arm_I";
  public static final String kArmDKey = "Arm_D";

  public static final String kTelePositionKey = "TelePosition";
  public static final String kTelePKey = "Tele_P";
  public static final String kTeleIKey = "Tele_I";
  public static final String kTeleDKey = "Tele_D";

  public static final String kWristPositionKey = "WristPosition";
  public static final String kWristPKey = "Wrist_P";
  public static final String kWristIKey = "Wrist_I";
  public static final String kWristDKey = "Wrist_D";

  public static final double kTelescopeMin = Units.inchesToMeters(28.25);
  public static final double kTelescopeMax = Units.inchesToMeters(73.50);

  // The P gain for the PID controller that drives this arm.
  public static final double kDefaultArmKp = 65.0;
  public static final double kDefaultArmKi = 0.0;
  public static final double kDefaultArmKd = 2.0;
  public static final double kDefaultArmSetpointDegrees = 200.0;
  public static final double kArmDeadband = 1.0;

  public static final double kDefaultTeleKp = 150.0;
  public static final double kDefaultTeleKi = 0.0;
  public static final double kDefaultTeleKd = 0.0;
  public static final double kDefaultTeleSetpointMeters = kTelescopeMin;

  public static final double kDefaultWristKp = 4.0;
  public static final double kDefaultWristKi = 0.0;
  public static final double kDefaultWristKd = 0.5;
  public static final double kDefaultWristSetpointDegrees = 0.0;

  public static final double kArmReduction = 200;
  public static final double kTelescopeReduction = 15.60;
  public static final double kArmMass = Units.lbsToKilograms(23.715); // Kilograms
  public static final double kWristMass = Units.lbsToKilograms(10); // Kilograms
  public static final double kTelescopeDrumRadius = Units.inchesToMeters(0.5);
  public static final double kArmLength = Units.inchesToMeters(28.25);
  public static final double kMinAngleRads = Units.degreesToRadians(-75);
  public static final double kMaxAngleRads = Units.degreesToRadians(255);
  public static final double kWristReduction = 20.0;
  public static final double kWristLength = Units.inchesToMeters(15);
  public static final double kWristMinAngleRads = Units.degreesToRadians(-90);
  public static final double kWristMaxAngleRads = Units.degreesToRadians(180);

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses) for arm rotation
  //  = elevator drum radius / 4096 pulses for telescope extension
  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
  public static final double kTelescopeEncoderDistPerPulse = 2.0 * kTelescopeDrumRadius / 4096;
  public static final double kWristEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  // position presets
  public static final double armStartingPos = 215.0;
  public static final double wristStartingPos = 155.0;
  public static final double teleStartingPos = Units.metersToInches(0.72);

  public static final double armL1Pos = 150.0;
  public static final double wristL1Pos = 90.0;

  public static final double armL2Pos = 135.0;
  public static final double wristL2Pos = 85.0;

  public static final double armL3Pos = 120.0;
  public static final double wristL3Pos = 90.0;
  public static final double teleL3Pos = Units.metersToInches(0.97);

  public static final double armL4Pos = 105.0;
  public static final double wristL4Pos = 97.0;
  public static final double teleL4Pos = Units.metersToInches(1.55);

}
