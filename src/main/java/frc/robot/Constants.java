// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort = 0;

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";
  public static final String kArmIKey = "ArmI";
  public static final String kArmDKey = "ArmD";

  public static final double kTelescopeMin = Units.inchesToMeters(28.25);
  public static final double kTelescopeMax = Units.inchesToMeters(73.50);

  // The P gain for the PID controller that drives this arm.
  public static final double kDefaultArmKp = 50.0;
  public static final double kDefaultArmKi = 0.0;
  public static final double kDefaultArmKd = 0.0;
  public static final double kDefaultArmSetpointDegrees = 200.0;
  public static final double kArmDeadband = 1.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  public static final double kArmReduction = 200;
  public static final double kTelescopeReduction = 15.06;
  public static final double kArmMass = Units.lbsToKilograms(23.715); // Kilograms
  public static final double kArmLength = Units.inchesToMeters(24.25);
  public static final double kMinAngleRads = Units.degreesToRadians(-75);
  public static final double kMaxAngleRads = Units.degreesToRadians(255);
}
