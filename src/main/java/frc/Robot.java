// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//hi this is yuji i shat myself

package frc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.wpilibj.ADIS16448_IMU; Analog Gyro
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.Log;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static final Log.Level LOG_LEVEL = Constants.LOG_ROBOT;
	private Log log = new Log(LOG_LEVEL, "Robot");


  /** Creating  Subsystems ********************************************/
  
  UsbCamera camera1;

  public static final OI oi = new OI();


  SendableChooser<Command> chooser = new SendableChooser<Command>();

  @Override
  public void robotInit() {
    
    log.add("Robot Init", Log.Level.TRACE);


    /* Autonomous Routines */
    //add more auto options here
    SmartDashboard.putData(chooser);

    camera1 = CameraServer.startAutomaticCapture(0);
    camera1.setResolution(320, 240);
    camera1.setFPS(15);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    //SmartDashboard.putNumber("FrontLeft Abs Radians", drivetrain.getFrontLeft().getAbsoluteEncoderRadians() * (180/Math.PI));
    //SmartDashboard.putNumber("FrontRight Abs Rotations", drivetrain.getFrontRight().getAbsoluteEncoderRadians() * (180/Math.PI));
    //SmartDashboard.putNumber("BackLeft Abs Rotations", drivetrain.getBackLeft().getAbsoluteEncoderRadians() * (180/Math.PI));
    //SmartDashboard.putNumber("BackRight Abs Rotations", drivetrain.getBackRight().getAbsoluteEncoderRadians() * (180/Math.PI));
		//SmartDashboard.putNumber("Gyro Angle", drivetrain.getGyroAngle()); // The current gyroscope angle

  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    m_autonomousCommand = chooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
