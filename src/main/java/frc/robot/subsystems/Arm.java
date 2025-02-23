// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
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

  private double m_wristKp = Constants.kDefaultWristKp;
  private double m_wristKi = Constants.kDefaultWristKi;
  private double m_wristKd = Constants.kDefaultWristKd;
  private double m_wristSetpointDegrees = Constants.kDefaultWristSetpointDegrees;

  private final DCMotor m_armGearbox = DCMotor.getNeoVortex(2);
  private final DCMotor m_telescopeGearbox = DCMotor.getNEO(2);
  private final DCMotor m_wristGearbox = DCMotor.getNeoVortex(1);

  // Standard classes for controlling our arm
  private final ProfiledPIDController m_controller = new ProfiledPIDController(m_armKp, m_armKi, m_armKd, new TrapezoidProfile.Constraints(2, 5));
  private final ProfiledPIDController m_teleController = new ProfiledPIDController(m_teleKp, m_teleKi, m_teleKd, new TrapezoidProfile.Constraints(5, 10));
  private final ProfiledPIDController m_wristController = new ProfiledPIDController(m_wristKp, m_wristKi, m_wristKd, new TrapezoidProfile.Constraints(2, 5));

  private final Encoder m_encoder =
      new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
  private final Encoder m_teleEncoder = 
      new Encoder(Constants.kTeleEncoderAChannel, Constants.kTeleEncoderBChannel);
  private final Encoder m_wristEncoder = 
      new Encoder(Constants.kWristEncoderAChannel, Constants.kWristEncoderBChannel);

  private final PWMSparkFlex m_motor1 = new PWMSparkFlex(Constants.kMotorPort);
  private final PWMSparkFlex m_motor2 = new PWMSparkFlex(Constants.kMotorPort + 1);
  private final PWMSparkFlex m_TeleMotor1 = new PWMSparkFlex(Constants.kMotorPort + 2);
  private final PWMSparkFlex m_TeleMotor2 = new PWMSparkFlex(Constants.kMotorPort + 3);
  private final PWMSparkFlex m_wristMotor = new PWMSparkFlex(Constants.kWristMotorPort + 4);

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
          VecBuilder.fill(Constants.kArmEncoderDistPerPulse).get(0) // Add noise with a std-dev of 1 tick
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
          VecBuilder.fill(Constants.kTelescopeEncoderDistPerPulse).get(0),
          0);
  
  private final SingleJointedArmSim m_wristSim = 
      new SingleJointedArmSim(
          m_wristGearbox,
          Constants.kWristReduction,
          SingleJointedArmSim.estimateMOI(Constants.kWristLength, Constants.kWristMass),
          Constants.kWristLength,
          Constants.kWristMinAngleRads,
          Constants.kWristMaxAngleRads,
          false,
          0,
          Constants.kWristEncoderDistPerPulse,
          VecBuilder.fill(Constants.kArmEncoderDistPerPulse).get(0)
      );

  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final EncoderSim m_teleEncoderSim = new EncoderSim(m_teleEncoder);
  private final EncoderSim m_wristEncoderSim = new EncoderSim(m_wristEncoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(120, 120);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 90, 23);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 20, -90));
  private final MechanismRoot2d m_CoralStation = m_mech2d.getRoot("Human Player Station", 60, 37.5);
  // private final MechanismLigament2d coralStation = 
  //     m_CoralStation.append(new MechanismLigament2d("Coral Station", 15, 125));

  // building the reef
  private final double reefXCord = 45.0;
  private final MechanismRoot2d m_reefRoot = m_mech2d.getRoot("Reef Base", reefXCord, 0);
  private final MechanismLigament2d reefTower = 
      m_reefRoot.append(new MechanismLigament2d("Reef Tower", 38.95 + 15.87, 90, 10, new Color8Bit(Color.kPurple)));
  private final MechanismRoot2d m_reefL1Root = m_mech2d.getRoot("L1 Root", reefXCord, 18);
  private final MechanismLigament2d m_L1 = 
      m_reefL1Root.append(new MechanismLigament2d("L1", 11.23 + 1.625, 0, 5, new Color8Bit(Color.kPurple)));
  private final MechanismRoot2d m_reefL2Root = m_mech2d.getRoot("L2 Root", reefXCord, 23.08);
  private final MechanismLigament2d m_L2 = 
      m_reefL2Root.append(new MechanismLigament2d("L2", 13.71, 35, 5, new Color8Bit(Color.kPurple)));
  private final MechanismRoot2d m_reefL3Root = m_mech2d.getRoot("L3 Root", reefXCord, 38.95);
  private final MechanismLigament2d m_L3 = 
      m_reefL3Root.append(new MechanismLigament2d("L3", 13.71, 35, 5, new Color8Bit(Color.kPurple)));
  private final MechanismRoot2d m_reefL4Root_A = m_mech2d.getRoot("L4a Root", reefXCord, 38.95 + 15.87);
  private final MechanismLigament2d m_L4a = 
      m_reefL4Root_A.append(new MechanismLigament2d("L4a", 14.23, 35, 10, new Color8Bit(Color.kPurple)));
  private final MechanismLigament2d m_L4b = 
      m_L4a.append(new MechanismLigament2d("L4b", 9, 90 - 35, 5, new Color8Bit(Color.kPurple)));

  // drivetrain
  private final MechanismLigament2d drivetrain = 
      m_armTower.append(new MechanismLigament2d("Drivetrain", 26, -90, 20, new Color8Bit(Color.kCyan)));

  // actual robot mechanisms
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              Units.metersToInches(Constants.kTelescopeMin),
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  private final MechanismLigament2d wrist = 
      m_arm.append(
          new MechanismLigament2d(
              "Wrist",
              Units.metersToInches(Constants.kWristLength),
              Units.radiansToDegrees(m_wristSim.getAngleRads()),
              3,
              new Color8Bit(Color.kGreen)));

  private double kArmLengthConversionFactor = m_arm.getLength() / Constants.kTelescopeMin;
      

  /** Subsystem constructor. */
  public Arm() {
    m_encoder.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);
    m_teleEncoder.setDistancePerPulse(Constants.kTelescopeEncoderDistPerPulse);
    m_wristEncoder.setDistancePerPulse(Constants.kWristEncoderDistPerPulse);

    m_controller.setTolerance(m_armDeadband);

    m_motor1.addFollower(m_motor2);
    m_TeleMotor1.addFollower(m_TeleMotor2);
    

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

    Preferences.initDouble(Constants.kWristPositionKey, m_wristSetpointDegrees);
    Preferences.initDouble(Constants.kWristPKey, m_wristKp);
    Preferences.initDouble(Constants.kWristIKey, m_wristKi);
    Preferences.initDouble(Constants.kWristDKey, m_wristKd);
  }

  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput((m_motor1.get() + m_motor2.get()) * RobotController.getBatteryVoltage());
    telescopeArm.setInput((m_TeleMotor1.get() + m_TeleMotor2.get()) * RobotController.getBatteryVoltage());
    m_wristSim.setInput(m_wristMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_wristSim.update(0.020);
    telescopeArm.update(0.020);
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    m_teleEncoderSim.setDistance(telescopeArm.getPositionMeters());
    m_wristEncoderSim.setDistance(m_wristSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_armSim.getCurrentDrawAmps()) 
            + telescopeArm.getCurrentDrawAmps()
            + m_wristSim.getCurrentDrawAmps());

    // SingleJointedArmSim.estimateMOI(telescopeArm.getPositionMeters(), Constants.kArmMass);

    // Update the Mechanism Arm angle and length based on the simulated arm angle and length
    m_arm.setLength((telescopeArm.getPositionMeters() * kArmLengthConversionFactor));
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    wrist.setAngle(Units.radiansToDegrees(m_wristSim.getAngleRads()));

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

    // aaaaand same thing for the wrist
    m_wristSetpointDegrees = Preferences.getDouble(Constants.kWristPositionKey, m_wristSetpointDegrees);
    if (m_wristKp != Preferences.getDouble(Constants.kWristPKey, m_wristKp)) {
      m_wristKp = Preferences.getDouble(Constants.kWristPKey, m_wristKp);
      m_wristController.setP(m_wristKp);
    }

    if (m_wristKi != Preferences.getDouble(Constants.kWristIKey, m_wristKi)) {
      m_wristKi = Preferences.getDouble(Constants.kWristIKey, m_wristKi);
      m_wristController.setI(m_wristKi);
    }

    if (m_wristKd != Preferences.getDouble(Constants.kWristDKey, m_wristKd)) {
      m_wristKd = Preferences.getDouble(Constants.kWristDKey, m_wristKd);
      m_wristController.setD(m_wristKd);
    }
  }

  public void setState(double armSetpoint, double wristSetpoint, double teleSetpoint) {

    loadPreferences();
    
    reachArmSetpoint(armSetpoint);
    reachWristSetpoint(wristSetpoint);
    reachTeleSetpoint(teleSetpoint);
    
  }

  /** Run the control loop to reach and maintain the setpoint from the preferences. */
  public void reachArmSetpoint(double setpoint) {
    var pidOutput =
        m_controller.calculate(
            m_encoder.getDistance(), Units.degreesToRadians(setpoint));
    m_motor1.setVoltage(pidOutput);
  }

  public void reachWristSetpoint(double setpoint) {
    var pidOutput = 
        m_wristController.calculate(
          m_wristEncoder.getDistance(), Units.degreesToRadians(setpoint));
    m_wristMotor.setVoltage(pidOutput);
  }

  public void reachTeleSetpoint(double setpoint) {
    var pidOutput = 
        m_teleController.calculate(
          m_teleEncoder.getDistance(), Units.inchesToMeters(setpoint));
    m_TeleMotor1.setVoltage(pidOutput);

  }

  public void extendArm() {
    loadPreferences();
    var pidOutput = m_teleController.calculate(
            m_teleEncoder.getDistance(), Constants.kTelescopeMax);
    m_TeleMotor1.setVoltage(pidOutput);
    
  }

  public void retractArm() {
    loadPreferences();
    var pidOutput = m_teleController.calculate(
            m_teleEncoder.getDistance(), Constants.kTelescopeMin);
    m_TeleMotor1.setVoltage(pidOutput);
  }

  public void stopArm() {
    m_motor1.set(0.0);
  }

  public void stopTelescope() {
    m_TeleMotor1.set(0.0);
  }

  public void stopWrist() {
    m_wristMotor.set(0.0);
  }

  @Override
  public void close() {
    m_motor1.close();
    m_TeleMotor1.close();
    m_encoder.close();
    m_mech2d.close();
    m_armPivot.close();
    m_arm.close();
  }
}
