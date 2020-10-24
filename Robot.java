/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.commands.ExampleCommand;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive
      = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  //private PWMTalonSRX left = new PWMTalonSRX(RobotMap.leftDrivePort), right = new PWMTalonSRX(RobotMap.rightDrivePort);
  private Encoder leftEncoder = new Encoder(0, 1) ;
  private Encoder rightEncoder = new Encoder(2, 3);
  public void DriveTrain() {
    //left.setInverted(true);
    leftEncoder.reset();
    rightEncoder.reset();
    leftEncoder.setDistancePerPulse(0.05);
    rightEncoder.setDistancePerPulse(0.05);
  }
  public void encoderDrive(double leftPower, double rightPower, 
  double leftDistance, double rightDistance){   
    if (leftPower < 0.05 && leftPower > -0.05){
      leftPower = 0;
    }
    if (rightPower < 0.05 && rightPower > -0.05){
      rightPower = 0;
    }
    leftEncoder.reset();
    rightEncoder.reset();
    double leftMovement = 0;
    double rightMovement = 0;
    while(leftDistance >= leftMovement & rightDistance >= rightMovement){
      if(leftDistance <= leftMovement & rightDistance <= rightMovement){
        m_robotDrive.tankDrive(leftPower, rightPower);
      }
    if(leftDistance <= leftMovement & rightDistance >= rightMovement){
        m_robotDrive.tankDrive(0, rightPower);
     }
      if(leftDistance >= leftMovement & rightDistance <= rightMovement){
        m_robotDrive.tankDrive(leftPower, 0);
      }
      leftMovement = leftEncoder.getDistance();
      rightMovement = rightEncoder.getDistance();
    }
    m_robotDrive.tankDrive(0, 0);
  }
  public void timeDrive(double leftPower, double rightPower, 
  double time){   
    m_timer.reset();
    m_timer.start();
    while (m_timer.get() < time) {
      m_robotDrive.tankDrive(leftPower, rightPower);
    }
    m_robotDrive.tankDrive(0,0);
  } 
  @Override
  public void robotInit() {
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    /*if (m_timer.get() < 2.0) {
      encoderDrive(.5, .5,100,100);
    } else {
      encoderDrive(-.5, -.5,100,100);
      //m_robotDrive.stopMotor(); // stop robot

    }*/
    //m_robotDrive.tankDrive(.5, .5);
    /*timeDrive(.5,.5,2);
    timeDrive(.5,-.5,2);
    timeDrive(-.5,.5,2);
    timeDrive(-.5,-.5,2);*/
    //encoderDrive(.5, .5,100000000,1000000000);
    m_timer.reset();
    m_timer.start();
    timeDrive(.5,.5,2.5);
    timeDrive(0,0,100000);
    /*double LE = leftEncoder.getDistance();
    double OLE = 0;
    double I = 0;
    while (m_timer.get() < 100000.0){
      LE = leftEncoder.getDistance();
      I++;
      if(I == 50){
        OLE = leftEncoder.getDistance();
      }
      if (I < 100){
        if ( LE <= 5){
          m_robotDrive.tankDrive(.5, -.5);
       }
        else {
         m_robotDrive.tankDrive(0, 0);
        }
      }
      if(I >= 100){
        if((LE-OLE) == 0){
          m_robotDrive.tankDrive(0, 0);
        }
        else {
          m_robotDrive.tankDrive(.3, .3);
        }
      }

    }*/
    /*while (m_timer.get() < 30.0){
      m_robotDrive.tankDrive(.5, .5);
    }*/

    //encoderDrive(.5, .5,100,100);
    //encoderDrive(-.5, -.5,100,100);
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
