/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// Import standard robot classes
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.XboxController;

// Import the LED Classes
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

// Import the REV Robotic (3rd party) Classes
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Import the Camera Classes
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;

// Imports no longer used (left here just in case)
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxPIDController;
//import com.revrobotics.SparkMaxLimitSwitch;

public class Robot extends TimedRobot {
  
  // Create motor instances
  private final CANSparkMax m_rightMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax m_leftMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax m_Lift = new CANSparkMax(5, MotorType.kBrushless); 
  private final CANSparkMax m_Intake = new CANSparkMax(6, MotorType.kBrushed);
  private final CANSparkMax m_Climb = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax m_ClimbFollower = new CANSparkMax(8, MotorType.kBrushless);
  
  // Create drive instance
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  
  // Create Xbox Controller instances
  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_driverController2 = new XboxController(1);
  
  // Create limit switche instances (only needed to display their value)
  // private final SparkMaxLimitSwitch m_ForwardLimit =  m_Lift.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  // private final SparkMaxLimitSwitch m_ReverseLimit =  m_Lift.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
   
  // Create camera instance
  UsbCamera camera;
  
  // Create led instances
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  
  // Define global variables need within the code
  double startTime = 0;
  double curTime = 0;
  double autoTimeElapsed = 0;
  double counter = 0;
  
  // private RelativeEncoder m_encoder =  m_Lift.getEncoder();
  // public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  //boolean DidRun = false;
  // true = raised
  /*//
 
  private boolean LiftState = true;
  private boolean ClimbState = true;
  private final void StartIntake() {
    m_Intake.set(0);
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    m_Intake.set(-0.3);
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    m_Intake.set(0.3);
    return;
  }
 
  // true = raised ; false = lowered
  private final void Lift() throws InterruptedException {
    if (LiftState == true) {
      m_Lift.set(-0.15); 
      Thread.sleep(2000);
      m_Lift.set(0);
      LiftState = false;}
    else {
      m_Lift.set(0.15); 
      Thread.sleep(2000);
      m_Lift.set(0);
      LiftState = true;
    }
    return;
  }

  // true = raised ; false = lowered
  private final void Climb() throws InterruptedException {
    if (ClimbState == true) {
      m_Climb.set(0.1); 
      Thread.sleep(2000);
      m_Climb.set(0);
      ClimbState = false;}
    else {
      m_Climb.set(-0.1); 
      Thread.sleep(2000);
      m_Climb.set(0);
      ClimbState = true;
    }
    return;
  }
 */

 /*// Create PID instances

    private SparkMaxPIDController m_pidControllerRight;
    private SparkMaxPIDController m_pidControllerLeft;

    private RelativeEncoder m_encoderRight;
    private RelativeEncoder m_encoderLeft;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

 */
 
  

  /**
  * This function runs when the robot is first started up.  Insert robot initialization code in 
  * this function
  */

  @Override

  public void robotInit() {
    
    m_led = new AddressableLED(1);
    m_ledBuffer = new AddressableLEDBuffer(60);
    for(var i = 0; i <  6; i++) {
      m_ledBuffer.setRGB(i, 255, 0, 0);
    }
   // m_pidController = m_Lift.getPIDController();
    

  /* Invert one side of the drivetrain so that positive voltages
  *  result in both sides moving forward. The practice robot's
  *  gearbox is constructed such that we have to invert the right side.
  *  Also set the follow motors to follow the respective lead motors
  */

    m_rightMotor.setInverted(true);
    m_leftFollower.follow(m_leftMotor);
    m_rightFollower.follow(m_rightMotor);

    

    m_ClimbFollower.setInverted(true);
    
    /*
    kP = 0.5; 
    kI = 0;
    kD = 0.5; 
    kIz = 0.1; 
    kFF = 0.1; 
    kMaxOutput = 1; 
    kMinOutput = -1;

   
    
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    */

    
  /*// initialze PID controller and encoder objects

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
    m_pidControllerLeft.setOutputRange(kMinOutput, kMaxOutput);*/

  /**
   * Smart Motion coefficients are set on a SparkMaxPIDController object
   * 
   * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
   *   the pid controller in Smart Motion mode
   * - setSmartMotionMinOutputVelocity() will put a lower bound in
   *   RPM of the pid controller in Smart Motion mode
   * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
   *   of the pid controller in Smart Motion mode
   * - setSmartMotionAllowedClosedLoopError() will set the max allowed
   *   error for the pid controller in Smart Motion mode
   */

    /*  int smartMotionSlot = 0;

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
      */
  
      //m_Lift.burnFlash();
      //Automatic
      
    // Start Microsoft camera capture
    m_Lift.burnFlash();
    camera = CameraServer.startAutomaticCapture(0);

    // Make sure camera is always on.
    camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

  }

  @Override

  public void autonomousInit() {
 
    startTime = Timer.getFPGATimestamp();
    m_leftMotor.setIdleMode(IdleMode.kCoast);
    m_rightMotor.setIdleMode(IdleMode.kCoast);
    m_leftFollower.setIdleMode(IdleMode.kCoast);
    m_rightFollower.setIdleMode(IdleMode.kCoast);
  }

  @Override

  public void autonomousPeriodic() {

    autoTimeElapsed = Timer.getFPGATimestamp() - startTime;
 
    if (autoTimeElapsed < 2.5) {
      m_Intake.set(-1);
    }
    else if (autoTimeElapsed < 4.5) {
      m_Intake.set(0);
      m_rightMotor.set(-0.3);
      m_leftMotor.set(-0.3);
      //m_robotDrive.tankDrive(-0.3, -0.3);
    } 
    else {
      m_rightMotor.set(0);
      m_leftMotor.set(0);
    } 
  } 


  @Override
  public void teleopInit() {
    //m_Intake.set((.3));
    m_ClimbFollower.follow(m_Climb, true);
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_leftFollower.setIdleMode(IdleMode.kBrake);
    m_rightFollower.setIdleMode(IdleMode.kBrake);
  }
  
  @Override
  public void teleopPeriodic() { 
    /*
    SmartDashboard.putNumber("Time", counter++);
    SmartDashboard.putNumber("trigger", m_driverController.getRightTriggerAxis());
    SmartDashboard.putBoolean("A button", m_driverController.getAButton());
    SmartDashboard.putBoolean("X button", m_driverController.getXButton());
    SmartDashboard.putNumber("pov", m_driverController.getPOV());
    SmartDashboard.putBoolean("FWD Limit", m_ForwardLimit.isPressed());
    SmartDashboard.putBoolean("REV Limit", m_ReverseLimit.isPressed());
    */

    /* Drive with tank drive or arcade drive using an xbox controller. The chosen driver
     driver will make the final choice. */

    //m_robotDrive.tankDrive(-m_driverController.getLeftY(), -m_driverController.getRightY());
    m_robotDrive.arcadeDrive(-m_driverController.getRightY(), m_driverController.getRightX(), true);
   
    
    
    // Positive = Raise
    if (  m_driverController2.getRightTriggerAxis() > .9){ 
      m_Climb.set(.5);
    }
    // Negative = Lower
    else if (m_driverController2.getLeftTriggerAxis() > .9){
      m_Climb.set(-.5);
    }
    //Turns off climb when no button is pressed
    else if (m_driverController2.getLeftTriggerAxis() == 0){
      m_Climb.set(0);
    }

    //lower lift when up arrow is pressed
    if (m_driverController2.getLeftBumper()) {
      m_Intake.set(.5);
    }

    //disable intake
    if (m_driverController2.getBButton()){
      m_Intake.set(0);
      m_Climb.set(0);
    }

    //lower lift when down arrow is pressed
    if(m_driverController2.getRightBumper()){
      m_Intake.set(-1);
    }

    //lower lift when pressed
    if( m_driverController2.getPOV() == 180){
      m_Lift.set(-.1);
    } 
     //higher lift when pressed
    else if(m_driverController2.getPOV() == 0){
      m_Lift.set(.25);
    }
    //Stop lift when no button is pressed
    else { 
      m_Lift.set(0);
    }

  
   
  }

  @Override

  public void testInit() {
    m_ClimbFollower.restoreFactoryDefaults();
  }
  
  @Override

  public void testPeriodic() {

    m_robotDrive.arcadeDrive(-m_driverController.getRightY(), m_driverController.getRightX());
  
    // Below controls Joe
    if (m_driverController.getRightBumper()){
    //  SmartDashboard.putNumber("bob", 1);
      m_ClimbFollower.set(.3); // Positive = Raised
    } else if (
      m_driverController.getRightTriggerAxis() > .9)
    {
    //  SmartDashboard.putNumber("bob", -1);
    m_ClimbFollower.set(-.3); // Negative = Lowered
    } else {
    m_ClimbFollower.set(0);
    //SmartDashboard.putNumber("bob", 0);
    }
    
    // Below controls Dificíl
    if (m_driverController.getLeftBumper()){
      m_Climb.set(-.3); // Positive = Raised
    } else if (
      m_driverController.getLeftTriggerAxis() > .9
    ) {
      m_Climb.set(.3); // Negative = Lowered
    } else {
      m_Climb.set(0);
    }

  }

  
}
