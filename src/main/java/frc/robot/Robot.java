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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


// END IMPORTS

public class Robot extends TimedRobot {
// Port Instantiation
  private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
  private static final int leftVictorPort = 0;
  private static final int rightVictorPort = 1;
  private static final int leftIntakeSparkPort = 2;
  private static final int rightIntakeSparkPort = 3;
  private static final int indexingSparkPort = 4;

// Joystick Ports
  private static final int kJoystickPort = 0;
  private static final int kJoystick2Port = 1;

// Button set-up
  private static final int bPowerCellIntake = 1;
  private static final int bPowerCellExpel = 2;

// Drive VictorSP's (commented out 1/6/20 for redesigned gyro code testing)
 VictorSP leftVictorSP = new VictorSP(leftVictorPort);
 VictorSP rightVictorSP = new VictorSP(rightVictorPort);
 
// Intake Spark's
  Spark leftIntakeSpark = new Spark(leftIntakeSparkPort);
  Spark rightIntakeSpark = new Spark(rightIntakeSparkPort);  

// Indexing Spark
  Spark indexingSpark = new Spark (indexingSparkPort);

// Gyro Instantiation 
  int P, I, D = 1;
  private static final double kP = 0.005; // propotional turning constant
  double angle;
  boolean turned = true;
  int mustTurnDegree = 0;
  private static final double kAngleSetpoint = 0.0;
  private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(kGyroPort);

//Joystick Instantiation
  private Joystick m_joystick = new Joystick(kJoystickPort);
  private Joystick m_joystick2 = new Joystick(kJoystick2Port);

// DriveTrain Creation
  private DifferentialDrive m_myRobot
    = new DifferentialDrive(leftVictorSP, rightVictorSP);    

// Encoder Creation
  public static Encoder elevatorEncoder;

// Pneumatic's Creation
  public static Compressor compressor;
  public static DoubleSolenoid indexPiston;

  // Auto Choices in Shuffleboard
  private static final String kAutoLine = "Drive Straight - Auto Line";
  private static final String kAutoLineRight = "Drive Straight - Turn Right";
  private static final String kAutoLineLeft = "Drive Straight - Turn Left";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

// Timer for autonomous
  public double timer = 0;

// END TIMED ROBOT METHOD

@Override
public void robotInit() {
// Joystick Creation
  m_joystick = new Joystick(0);
  m_joystick2 = new Joystick(1);

// Encoder Instantiation
  elevatorEncoder = new Encoder(4, 5, true, Encoder.EncodingType.k4X);
  elevatorEncoder.setDistancePerPulse((Math.PI * 1.804) / 192);

// Pneumatics Instantiation
  compressor = new Compressor(1);
  indexPiston = new DoubleSolenoid(4, 5);

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
// Post Encoder Distance to Shuffleboard
  SmartDashboard.putNumber("Encoder Distance", elevatorEncoder.getDistance());
}

// END ROBOT PERIODIC METHOD

@Override
public void autonomousInit() {
  m_autoSelected = m_chooser.getSelected();
}

// END AUTONOMOUS INIT METHOD

@Override
public void autonomousPeriodic() {

// Code for auto choices
  switch (m_autoSelected) {
    case kAutoLine:
      default:
      if (Timer.getMatchTime()<timer+5){
        m_myRobot.tankDrive(0.25, -0.25);
      }
      else{
        m_myRobot.tankDrive(0, 0);
      }
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

  // Intake/Outtake Control Statements
  if (m_joystick2.getRawButton(bPowerCellIntake)) {
    leftIntakeSpark.set(0.5);
    rightIntakeSpark.set(-0.5);
  } 
  else if (m_joystick2.getRawButton(bPowerCellExpel)) {
    leftIntakeSpark.set(-1);
    rightIntakeSpark.set(1);
  }
  else{
    leftIntakeSpark.set(0);
    rightIntakeSpark.set(0);
  }
}

// END TELEOP PERIODIC

@Override
public void testPeriodic() {
}
}

// END TEST PERIODIC METHOD & ROBOT PROJECT