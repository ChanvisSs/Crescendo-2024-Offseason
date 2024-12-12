// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import frc.lib.Log;
import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {

  public static final double JOYSTICK_DRIVE_SCALE = Constants.JOYSTICK_DRIVE_SCALE;
	public static final double JOYSTICK_DRIVE_SCALE_LOW = Constants.JOYSTICK_DRIVE_SCALE_LOW;
  public static boolean isLowScale = false;

  Log log = new Log(Constants.LOG_OI, "OI");

  public static double CURRENT_DRIVE_SCALE = JOYSTICK_DRIVE_SCALE;
  public static final GenericHID driverController = new GenericHID(Constants.DRIVER_XBOX_USB_PORT);
  public static final GenericHID gunnerController = new GenericHID(Constants.GUNNER_XBOX_USB_PORT);

  public boolean getLeftJoyStickUp(GenericHID gunnerController){
	return gunnerController.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) >= 0.5;
  }

  public boolean getRightJoyStickUp(GenericHID gunnerController){
	return gunnerController.getRawAxis(Constants.RIGHT_VERTICAL_JOYSTICK_AXIS) >= 0.5;
  }

  public boolean getLeftJoyStickDown(GenericHID gunnerController){
	return gunnerController.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) <= -0.5;
  }

  public boolean getRightJoyStickDown(GenericHID gunnerController){
	return gunnerController.getRawAxis(Constants.RIGHT_VERTICAL_JOYSTICK_AXIS) <= -0.5;
  }

  public boolean getRightTrigger(GenericHID gunnerController){
		return gunnerController.getRawAxis(Constants.RIGHT_TRIGGER_AXIS) >= 0.95 ;
	}


  public OI() {
    configureBindings();
	}  
   
  public void setNormalScale() {
		CURRENT_DRIVE_SCALE = JOYSTICK_DRIVE_SCALE;
		isLowScale = false;
	}
	public void setLowScale(){
		CURRENT_DRIVE_SCALE = JOYSTICK_DRIVE_SCALE_LOW;
		isLowScale = true;
	}
	public void toggleScale() {

		if (isLowScale){

			setNormalScale();
		} else {

			setLowScale();
		}
	}

  public double getXSpeed() {
		double joystickValue = driverController.getRawAxis(1);
		// double joystickValue = driverController.getRightY(); // this would be for a flight joystick
		if (Math.abs(joystickValue) < 0.07) { // This is our deadband
			return 0.0;
		}
		else {
			return joystickValue * Constants.kPhysicalMaxSpeedMetersPerSecond * CURRENT_DRIVE_SCALE * 1; // Multiply by -1 reverses the direction
		}	
	}
	public double getYSpeed() {
		// double joystickValue = joyRight.getX(); // this would be for a flight joystick
		double joystickValue = driverController.getRawAxis(0);
		if (Math.abs(joystickValue) < 0.07) { // This is our deadband
			return 0.0;
		}
		else {
			return joystickValue * Constants.kPhysicalMaxSpeedMetersPerSecond * CURRENT_DRIVE_SCALE * 1; // Multiply by -1 reverses the direction, 0.5 to reduce speed
		}	
	}
	public double getZSpeed() {
		// double joystickValue = joyLeft.getX(); // this would be for a flight joystick
		double joystickValue = driverController.getRawAxis(4);
		if (Math.abs(joystickValue) < 0.07) { // This is our deadband
			return 0.0;
		}
		else {
			return joystickValue * Constants.kPhysicalMaxTurningSpeedRadiansPerSecond * CURRENT_DRIVE_SCALE * -1; // Multiply by -1 reverses the direction
		}	
	}


  private void configureBindings() {}

}

