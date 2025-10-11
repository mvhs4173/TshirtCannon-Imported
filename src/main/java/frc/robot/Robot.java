// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private int shotCount = 0; //Increments with each shot to use different barrels with the same button
  private final double pulseDuration = 1.0; //length (seconds) for solenoids to fire
  private double boostSpeed = 1.0; //speed to drive when boost (right bumper) is pressed
  private double normalSpeed = 0.2; //normal speed to drive (when boost is not pressed)

  XboxController m_Controller = new XboxController(0);

  SparkMax m_frontLeftMotor = new SparkMax(0, MotorType.kBrushed);
  SparkMax m_frontRightMotor = new SparkMax(1, MotorType.kBrushed);
  SparkMax m_backLeftMotor = new SparkMax(2, MotorType.kBrushed);
  SparkMax m_backRightMotor = new SparkMax(3, MotorType.kBrushed);

  Solenoid m_BarrelOneSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  Solenoid m_BarrelTwoSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
  Solenoid m_BarrelThreeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);
  Solenoid m_BarrelFourSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 3);

  Compressor m_Compressor = new Compressor(PneumaticsModuleType.REVPH);

  Encoder m_frontLeftEncoder = new Encoder(0, 1);
  Encoder m_frontRightEncoder = new Encoder(2,3);
  Encoder m_backLeftEncoder = new Encoder(4,5);
  Encoder m_backRightEncoder = new Encoder(6,7);

  MecanumDrive m_MecanumDrive = new MecanumDrive(m_frontLeftMotor,
   m_backLeftMotor, m_frontRightMotor, m_backRightMotor);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_Compressor.enableDigital();
    m_BarrelOneSolenoid.setPulseDuration(pulseDuration);
    //going to invert the right side motors next time
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double frontLeftRate = m_frontLeftEncoder.getRate();
    double frontRightRate = m_frontRightEncoder.getRate();
    double backLeftRate = m_backLeftEncoder.getRate();
    double backRightRate = m_backRightEncoder.getRate();

    SmartDashboard.putNumber("frontLeftRate", frontLeftRate);
    SmartDashboard.putNumber("frontRightRate", frontRightRate);
    SmartDashboard.putNumber("backLeftRate", backLeftRate);
    SmartDashboard.putNumber("backRightRate", backRightRate);


    double forwardBackward = -m_Controller.getLeftY();
    double leftRight = m_Controller.getLeftX();
    double rotation = m_Controller.getRightX();
    boolean boost = m_Controller.getRightBumperButton();
    forwardBackward *= boost ? boostSpeed : normalSpeed;
    leftRight *= boost ? boostSpeed : normalSpeed;
    rotation *= boost ? boostSpeed : normalSpeed;
    m_MecanumDrive.driveCartesian(forwardBackward,
     leftRight, rotation);

    if(m_Controller.getAButton()){
      if(shotCount == 0){
        m_BarrelOneSolenoid.startPulse();
      } else if(shotCount == 1){
        m_BarrelTwoSolenoid.startPulse();
      } else if(shotCount == 2){
        m_BarrelThreeSolenoid.startPulse();
      } else if(shotCount == 3){
        m_BarrelFourSolenoid.startPulse();
      }

      shotCount++;
      shotCount %= 4;


    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
