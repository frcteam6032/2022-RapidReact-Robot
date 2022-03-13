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
  
  
  /** robotInit: This function runs when the robot is first started up. */
  @Override
  public void robotInit() {
    
    m_led = new AddressableLED(1);
    m_ledBuffer = new AddressableLEDBuffer(60);
    for(var i = 0; i <  6; i++) {
      m_ledBuffer.setRGB(i, 255, 0, 0);
    }
    

    /* Invert one side of the drivetrain so that positive voltages
    *  result in both sides moving forward. The practice robot's
    *  gearbox is constructed such that we have to invert the right side.
    *  Also set the follow motors to follow the respective lead motors
    */

    m_rightMotor.setInverted(true);
    m_leftFollower.follow(m_leftMotor);
    m_rightFollower.follow(m_rightMotor);
      
    // Start Microsoft camera capture
    camera = CameraServer.startAutomaticCapture(0);

    // Make sure camera is always on.
    camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

  }

  
  /** autonomousInit: This function is called once when autonomous is enabled.  */
  @Override
  public void autonomousInit() {
 
    // Get the initial time value
    startTime = Timer.getFPGATimestamp();
    
    // Set the drive motors to coast mode
    m_leftMotor.setIdleMode(IdleMode.kCoast);
    m_rightMotor.setIdleMode(IdleMode.kCoast);
    m_leftFollower.setIdleMode(IdleMode.kCoast);
    m_rightFollower.setIdleMode(IdleMode.kCoast);
  }

  
  /** autonomousPeriodic: This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /** 
     * Single autonomous play:
     *  1. Immediately shoot the ball for 2.5 seconds
     *  2. At 2.5 seconds, turn off shoot and move in reverse
     *  3. At 4.5 seconds, stop the robot
    */

    // Update the time elapsed value
    autoTimeElapsed = Timer.getFPGATimestamp() - startTime;
 
    // Process the automous play stages
    if (autoTimeElapsed < 2.5) {
      // Stage 1: Shooting
      m_Intake.set(-1);
    }
    else if (autoTimeElapsed < 4.5) {
      // Stage 2: Reverse the robot
      m_Intake.set(0);
      m_rightMotor.set(-0.3);
      m_leftMotor.set(-0.3);
    } 
    else {
      // Stage 3: Stop the robot
      m_rightMotor.set(0);
      m_leftMotor.set(0);
    }
    
    /** 
     * Note: DriveTrain Motor Safety errors are thrown during this section.
     * The motors get set to 0, but then reset to -0.3.  
     * With the motors in "coast" mode, this issue is not noticeable 
     * 
     * Possible fix by replacing the m_rightMotor.set(x) and m_leftMotor.set(x)
     * commands with an equivalent m_robotDrive.arcadeDrive command, such as
     * m_robotDrive.arcadeDrive(x, 0, false)
     * 
     * This fix has been tested on deployed code.
    */    
  } 

  
  /** teleopInit: This function is called once when teleop is enabled.  */
  @Override
  public void teleopInit() {
    //m_Intake.set((.3));
    
    // Enables climb follower (resets to factory in test mode)
    m_ClimbFollower.follow(m_Climb, true);  // "True" sets follower as inverted
    
    // Set the drive motors to brake mode
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_leftFollower.setIdleMode(IdleMode.kBrake);
    m_rightFollower.setIdleMode(IdleMode.kBrake);
  }
  
  
  /** teleopPeriodic: This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() { 
    
    /*
    // Smart Dashboard display commands for debugging
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
  
    /** 
     * Look for trigger inputs and set climber motors accordingly
     *    Right Trigger: Climber Motor Forward
     *    Left Trigger: Climber Motor Reverse
     *    No Left Trigger: Climber Motor Off
     */
    if (  m_driverController2.getRightTriggerAxis() > .9){ 
      m_Climb.set(.5); // Positive = Raise
    }
    else if (m_driverController2.getLeftTriggerAxis() > .9){
      m_Climb.set(-.5); // Negative = Lower
    }
    else if (m_driverController2.getLeftTriggerAxis() == 0){
      m_Climb.set(0); // Turns off climb when left button is not pressed
      // TODO: This should probably just be an else, not elseif
    }

    // If left bumper is ever pressed, enable intake motor
    if (m_driverController2.getLeftBumper()) {
      m_Intake.set(.5);
    }

    // If B is ever pressed, disable intake and climb 
    if (m_driverController2.getBButton()){
      m_Intake.set(0);
      m_Climb.set(0);
    }

    // If right bumper is ever pressed, shoot intake
    if(m_driverController2.getRightBumper()){
      m_Intake.set(-1);
    }

    // If D-pad is ever pressed down, shoot intake
    
    /** 
     * Look for D-Pad inputs and set lift motor accordingly
     *    Down: Lower Lift Arm
     *    Up: Raise Lift Arm
     *    Nothing: Lift Arm Off
     */
    if( m_driverController2.getPOV() == 180){
      m_Lift.set(-.1);  // Lower lift arm
    } 
    else if(m_driverController2.getPOV() == 0){
      m_Lift.set(.25); // Raise lift arm
    }
    else { 
      m_Lift.set(0);  //Stop lift arm
    }
   
  }

  
  /** testInit: This function is called once when test is enabled.  */
  @Override
  public void testInit() {
    // Reset the ClimbFollower so it can be run independently in test mode
    m_ClimbFollower.restoreFactoryDefaults();
    m_ClimbFollower.setInverted(true);  //THIS NEEDS TO BE TESTED FOR DIRECTION
  }
  
  
  /** testPeriodic: This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    // Enable operator control for driving
    m_robotDrive.arcadeDrive(-m_driverController.getRightY(), m_driverController.getRightX());
  
    /**
     * Climbing arm adjustments with right/left bumper and triggers
     *    Right Bumper: Raise Right Arm
     *    Right Trigger: Lower Right Arm
     *
     *    Left Bumper: Raise Left Arm
     *    Left Trigger: Lower Left Arm
     */
    // Below controls Joe
    if (m_driverController.getRightBumper()){
      m_ClimbFollower.set(0.3); // Positive = Raised
    } else if (m_driverController.getRightTriggerAxis() > .9){
      m_ClimbFollower.set(-0.3); // Negative = Lowered
    } else {
      m_ClimbFollower.set(0);
    }
    
    // Below controls Dificil
    if (m_driverController.getLeftBumper()){
      m_Climb.set(0.3); // Positive = Raised
    } else if (m_driverController.getLeftTriggerAxis() > .9){
      m_Climb.set(-0.3); // Negative = Lowered
    } else {
      m_Climb.set(0);
    }
  }  
}