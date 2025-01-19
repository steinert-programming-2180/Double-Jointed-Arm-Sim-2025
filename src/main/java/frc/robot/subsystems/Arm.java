// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Arm implements AutoCloseable {
  // The P gain for the PID controller that drives this arm.
  private double m_armKp = Constants.kDefaultArmKp;
  private double m_armKi = Constants.kDefaultArmKi;
  private double m_armKd = Constants.kDefaultArmKd;
  private double m_armSetpointDegrees = Constants.kDefaultArmSetpointDegrees;
  private double m_armDeadband = Constants.kArmDeadband;

  private double m_teleKp = Constants.kDefaultTeleKp;
  private double m_teleKi = Constants.kDefaultTeleKi;
  private double m_teleKd = Constants.kDefaultTeleKd;
  private double m_teleSetpointMeters = Constants.kDefaultTeleSetpointMeters;

  // The arm gearbox represents a gearbox containing two NEO motor.
  private final DCMotor m_armGearbox = DCMotor.getNEO(2);
  private final DCMotor m_telescopeGearbox = DCMotor.getNEO(1);

  // Standard classes for controlling our arm
  private final PIDController m_controller = new PIDController(m_armKp, m_armKi, m_armKd);
  private final PIDController m_teleController = new PIDController(m_teleKp, m_teleKi, m_teleKd);
  private final Encoder m_encoder =
      new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
  private final Encoder m_teleEncoder = 
      new Encoder(Constants.kTeleEncoderAChannel, Constants.kTeleEncoderBChannel);
  private final PWMSparkFlex m_motor = new PWMSparkFlex(Constants.kMotorPort);
  private final PWMSparkFlex m_TeleMotor = new PWMSparkFlex(Constants.kTeleMotorPort);

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          m_armGearbox,
          Constants.kArmReduction,
          SingleJointedArmSim.estimateMOI(Constants.kTelescopeMin, Constants.kArmMass),
          Constants.kTelescopeMin,
          Constants.kMinAngleRads,
          Constants.kMaxAngleRads,
          true,
          0,
          Constants.kArmEncoderDistPerPulse,
          0.0 // Add noise with a std-dev of 1 tick
          );

  private final ElevatorSim telescopeArm = 
      new ElevatorSim(
          m_telescopeGearbox,
          Constants.kTelescopeReduction,
          Constants.kWristMass,
          Constants.kTelescopeDrumRadius,
          Constants.kTelescopeMin,
          Constants.kTelescopeMax,
          true,
          Constants.kTelescopeMin,
          0.0,
          0);

  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final EncoderSim m_teleEncoderSim = new EncoderSim(m_teleEncoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(120, 120);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 60, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 45, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              Units.metersToInches(Constants.kTelescopeMin),
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  private double kArmLengthConversionFactor = m_arm.getLength() / Constants.kTelescopeMin;
      

  /** Subsystem constructor. */
  public Arm() {
    m_encoder.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);
    m_teleEncoder.setDistancePerPulse(Constants.kTelescopeEncoderDistPerPulse);

    m_controller.setTolerance(m_armDeadband);

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));

    // Set the Arm position setpoint and P constant to Preferences if the keys don't already exist
    Preferences.initDouble(Constants.kArmPositionKey, m_armSetpointDegrees);
    Preferences.initDouble(Constants.kArmPKey, m_armKp);
    Preferences.initDouble(Constants.kArmIKey, m_armKi);
    Preferences.initDouble(Constants.kArmDKey, m_armKd);

    Preferences.initDouble(Constants.kTelePositionKey, m_teleSetpointMeters);
    Preferences.initDouble(Constants.kTelePKey, m_teleKp);
    Preferences.initDouble(Constants.kTeleIKey, m_teleKi);
    Preferences.initDouble(Constants.kTeleDKey, m_teleKd);
  }

  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());
    telescopeArm.setInput(m_TeleMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);
    telescopeArm.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    m_teleEncoderSim.setDistance(telescopeArm.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()) + telescopeArm.getCurrentDrawAmps());

    // SingleJointedArmSim.estimateMOI(telescopeArm.getPositionMeters(), Constants.kArmMass);

    // Update the Mechanism Arm angle and length based on the simulated arm angle and length
    m_arm.setLength((telescopeArm.getPositionMeters() * kArmLengthConversionFactor));
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

    Preferences.setDouble("Arm Current Load", m_armSim.getCurrentDrawAmps());

    
  }

  /** Load setpoint and kP from preferences. */
  public void loadPreferences() {
    // Read Preferences for Arm setpoint and kP on entering Teleop
    m_armSetpointDegrees = Preferences.getDouble(Constants.kArmPositionKey, m_armSetpointDegrees);
    if (m_armKp != Preferences.getDouble(Constants.kArmPKey, m_armKp)) {
      m_armKp = Preferences.getDouble(Constants.kArmPKey, m_armKp);
      m_controller.setP(m_armKp);
    }

    if (m_armKi != Preferences.getDouble(Constants.kArmIKey, m_armKi)) {
      m_armKi = Preferences.getDouble(Constants.kArmIKey, m_armKi);
      m_controller.setI(m_armKi);
    }

    if (m_armKd != Preferences.getDouble(Constants.kArmDKey, m_armKd)) {
      m_armKd = Preferences.getDouble(Constants.kArmDKey, m_armKd);
      m_controller.setD(m_armKd);
    }

    // same thing, except it's for the telescope this time
    m_teleSetpointMeters = Preferences.getDouble(Constants.kTelePositionKey, m_teleSetpointMeters);
    if (m_teleKp != Preferences.getDouble(Constants.kTelePKey, m_teleKp)) {
      m_teleKp = Preferences.getDouble(Constants.kTelePKey, m_teleKp);
      m_teleController.setP(m_teleKp);
    }

    if (m_teleKi != Preferences.getDouble(Constants.kTeleIKey, m_teleKi)) {
      m_teleKi = Preferences.getDouble(Constants.kTeleIKey, m_teleKi);
      m_teleController.setI(m_teleKi);
    }

    if (m_teleKd != Preferences.getDouble(Constants.kTeleDKey, m_teleKd)) {
      m_teleKd = Preferences.getDouble(Constants.kTeleDKey, m_teleKd);
      m_teleController.setD(m_teleKd);
    }
  }

  /** Run the control loop to reach and maintain the setpoint from the preferences. */
  public void reachSetpoint() {
    loadPreferences();
    var pidOutput =
        m_controller.calculate(
            m_encoder.getDistance(), Units.degreesToRadians(m_armSetpointDegrees));
    m_motor.setVoltage(pidOutput);
  }

  public void extendArm() {
    loadPreferences();
    var pidOutput = m_teleController.calculate(
            m_teleEncoder.getDistance(), Constants.kTelescopeMax);
    m_TeleMotor.setVoltage(pidOutput);
    
  }

  public void retractArm() {
    loadPreferences();
    var pidOutput = m_teleController.calculate(
            m_teleEncoder.getDistance(), Constants.kTelescopeMin);
    m_TeleMotor.setVoltage(pidOutput);
  }

  public void stopArm() {
    m_motor.set(0.0);
  }

  public void stopTelescope() {
    m_TeleMotor.set(0.0);
  }

  @Override
  public void close() {
    m_motor.close();
    m_encoder.close();
    m_mech2d.close();
    m_armPivot.close();
    m_controller.close();
    m_arm.close();
  }
}
