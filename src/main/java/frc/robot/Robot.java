/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Spark;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.*;

// END IMPORTS

public class Robot extends TimedRobot {
// Port Instantiation
  private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
  private static final int leftVictorPort = 0;
  private static final int rightVictorPort = 1;
  private static final int leftIntakeSparkPort = 2;
  private static final int rightIntakeSparkPort = 3;

// Joystick Ports
  private static final int kJoystickPort = 0;
  private static final int kJoystick2Port = 1;

// Drive VictorSP's
//  VictorSP leftVictorSP = new VictorSP(leftVictorPort);
//  VictorSP rightVictorSP = new VictorSP(rightVictorPort);
 
// Intake Spark's
  Spark leftIntakeSpark = new Spark(leftIntakeSparkPort);
  Spark rightIntakeSpark = new Spark(rightIntakeSparkPort);  

// New Gyro Instantiation 
  int P, I, D = 1;
  private static final double kP = 0.005; // propotional turning constant
  double angle;
  boolean turned = true;
  int mustTurnDegree = 0;
  private static final double kAngleSetpoint = 0.0;

// Drivetrain (with VictorSP's)
  //DifferentialDrive m_myRobot = new DifferentialDrive(leftVictorSP, rightVictorSP);
  private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(kGyroPort);
  private Joystick m_joystick = new Joystick(kJoystickPort);
  private Joystick m_joystick2 = new Joystick(kJoystick2Port);

// Gyro Stuff (creates DriveTrain as well)
  private DifferentialDrive m_myRobot
    = new DifferentialDrive(new VictorSP(leftVictorPort),
    new VictorSP(rightVictorPort));

// Auto Choices in Shuffleboard
  private static final String kAutoLine = "Drive Straight - Auto Line";
  private static final String kAutoLineRight = "Drive Straight - Turn Right";
  private static final String kAutoLineLeft = "Drive Straight - Turn Left";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

// END TIMED ROBOT METHOD

@Override
public void robotInit() {
// Joystick Creation
  m_joystick = new Joystick(0);
  m_joystick2 = new Joystick(1);

// Camera Instantiation
  CameraServer camera = CameraServer.getInstance();
  VideoSource usbCam = camera.startAutomaticCapture("cam0", 0);
  usbCam.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);
  CameraServer camera2 = CameraServer.getInstance();
  VideoSource usbCam2 = camera2.startAutomaticCapture("cam1", 1);
  usbCam2.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);  

// Gyro
  m_gyro.calibrate();

// Creating Dropdown Choices in Shuffleboard
  m_chooser.setDefaultOption("Drive Straight - Auto Line", kAutoLine);
  m_chooser.addOption("Drive Straight - Turn Right", kAutoLineRight);
  m_chooser.addOption("Drive Straight - Turn Left", kAutoLineLeft);
  SmartDashboard.putData("Auto Chooser", m_chooser);
}

// END ROBOT INIT METHOD

@Override
public void robotPeriodic() {
}

// END ROBOT PERIODIC METHOD

@Override
public void autonomousInit() {
}

// END AUTONOMOUS INIT METHOD

@Override
public void autonomousPeriodic() {

// Code for auto choices
  switch (m_autoSelected) {
    case kAutoLine:
      default:
  //code goes here
      break;
    case kAutoLineRight:
  //code goes here
      break;
    case kAutoLineLeft:
  //code goes here  
      break;

}
}

// END AUTONOMOUS PERIODIC METHOD

//Gyro Math Method
public void turnDegrees(int degree) {
  if(turned)return;
  angle = m_gyro.getAngle() % 360;
  if(angle-10 > degree)m_myRobot.arcadeDrive(0.8, (angle - degree)*kP);
  else if(angle+10 < degree)m_myRobot.arcadeDrive(0.8, (angle + degree)*kP);
  else turned = true;
}

// END GYRO MATH METHOD

@Override
public void teleopPeriodic() {
//Gyro Math (tested & working as of 2/9/19) (old math is commented out as of 1/6/20)
  // m_myRobot.arcadeDrive(m_joystick.getY()*0.8, m_joystick.getX()*0.8);

  // if(m_joystick.getRawButton(1))turned = true;
  // if(m_joystick.getPOV() != -1){
  // turned = false;
  // mustTurnDegree = m_joystick.getPOV();
  // }
  // if(!turned)turnDegrees(mustTurnDegree);
    
  /**
	 * The motor speed is set from the joystick while the RobotDrive turning
	 * value is assigned from the error between the setpoint and the gyro angle.
	 */
  double turningValue = (kAngleSetpoint - m_gyro.getAngle()) * kP;
  // Invert the direction of the turn if we are going backwards
  turningValue = Math.copySign(turningValue, m_joystick.getY());
  m_myRobot.arcadeDrive(m_joystick.getY(), turningValue);
}

// END TELEOP PERIODIC

@Override
public void testPeriodic() {
}

//
protected void execute() {
  SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());

}
}

// END TEST PERIODIC METHOD & ROBOT PROJECT
