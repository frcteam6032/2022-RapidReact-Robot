/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Before Running:
 * Open shuffleBoard, select File->Load Layout and select the 
 * shuffleboard.json that is in the root directory of this example
 */

import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;


public class Robot extends TimedRobot {
// Create motor instances

  private final CANSparkMax m_rightMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax m_leftMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(4, MotorType.kBrushless);

  // Create PID instances

  private SparkMaxPIDController m_pidControllerRight;
  private SparkMaxPIDController m_pidControllerLeft;

  private RelativeEncoder m_encoderRight;
  private RelativeEncoder m_encoderLeft;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  // Create drive instance

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  
  // Create Xbox Controller instance

  private final XboxController m_driverController = new XboxController(0);
  
  // Define global variables
  private double startTime;
  UsbCamera camera;

  @Override

  public void robotInit() {

    /* Invert one side of the drivetrain so that positive voltages
       result in both sides moving forward. The practice robot's
       gearbox is constructed such that we have to invert the right side. */

      m_rightMotor.setInverted(true);
      m_leftFollower.follow(m_leftMotor);
      m_rightFollower.follow(m_rightMotor);

      // initialze PID controller and encoder objects

      m_pidControllerRight = m_rightMotor.getPIDController();
      m_encoderRight = m_rightMotor.getEncoder();

      m_pidControllerLeft = m_leftMotor.getPIDController();
      m_encoderLeft = m_leftMotor.getEncoder();

     
      // PID coefficients

      kP = 5e-5; 
      kI = 1e-6;
      kD = 0; 
      kIz = 0; 
      kFF = 0.000156; 
      kMaxOutput = 1; 
      kMinOutput = -1;
      maxRPM = 5700;

     // Smart Motion Coefficients

     maxVel = 2000; // rpm
     maxAcc = 1500;
 
     // set PID coefficients

     m_pidControllerRight.setP(kP);
     m_pidControllerRight.setI(kI);
     m_pidControllerRight.setD(kD);
     m_pidControllerRight.setIZone(kIz);
     m_pidControllerRight.setFF(kFF);
     m_pidControllerRight.setOutputRange(kMinOutput, kMaxOutput);

     m_pidControllerLeft.setP(kP);
     m_pidControllerLeft.setI(kI);
     m_pidControllerLeft.setD(kD);
     m_pidControllerLeft.setIZone(kIz);
     m_pidControllerLeft.setFF(kFF);
     m_pidControllerLeft.setOutputRange(kMinOutput, kMaxOutput);

     /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */

      int smartMotionSlot = 0;
      m_pidControllerRight.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
      m_pidControllerRight.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
      m_pidControllerRight.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
      m_pidControllerRight.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

      m_pidControllerLeft.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
      m_pidControllerLeft.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
      m_pidControllerLeft.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
      m_pidControllerLeft.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

      // display PID coefficients on SmartDashboard

      SmartDashboard.putNumber("P Gain", kP);
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("I Zone", kIz);
      SmartDashboard.putNumber("Feed Forward", kFF);
      SmartDashboard.putNumber("Max Output", kMaxOutput);
      SmartDashboard.putNumber("Min Output", kMinOutput);

      // display Smart Motion coefficients

      SmartDashboard.putNumber("Max Velocity", maxVel);
      SmartDashboard.putNumber("Min Velocity", minVel);
      SmartDashboard.putNumber("Max Acceleration", maxAcc);
      SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
      SmartDashboard.putNumber("Set Position", 0);
      SmartDashboard.putNumber("Set Velocity", 0);
  
      // button to toggle between velocity and smart motion modes

      SmartDashboard.putBoolean("Mode", true);
  

    // Start Microsoft camera capture
    
    camera = CameraServer.startAutomaticCapture(0);

    // Make sure camera is always on

    camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

  }

  @Override

  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override

  public void autonomousPeriodic() {
  double curTime = Timer.getFPGATimestamp();
double speed = 0.3;
double time = 1;
  if (curTime - startTime < time) {
    m_rightMotor.set(speed);
    m_leftMotor.set(speed);
   }
    else {
    m_rightMotor.set(0);
    m_leftMotor.set(0);
    
  }
  } 

  @Override

  public void teleopInit() {

  } 

  @Override

  public void teleopPeriodic() { 
  
  /* Drive with tank drive or arcade drive using an xbox controller. The chosen driver
     driver will make the final choice. */

   m_robotDrive.tankDrive(-m_driverController.getLeftY(), -m_driverController.getRightY());
   // m_robotDrive.arcadeDrive(-m_driverController.getRightY(), m_driverController.getRightX());
  }

  @Override

  public void testInit() {

  }
  
  @Override

  public void testPeriodic() {

  }
}
