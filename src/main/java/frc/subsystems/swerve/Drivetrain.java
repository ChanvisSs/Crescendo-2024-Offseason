package frc.subsystems.swerve;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;

import frc.lib.Log;
import frc.Constants;

/** Drivetrain ****************************************************************
 * The Swerve Drivetrain subsystem of the robot. */
public class Drivetrain extends SubsystemBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = Constants.LOG_DRIVETRAIN;
	private static final double TRACK_WIDTH = Constants.TRACK_WIDTH;
	private static final double WHEEL_BASE = Constants.WHEEL_BASE;

	private final SwerveModule frontLeft = new SwerveModule(
		Constants.kFrontLeftDriveMotorPort,
		Constants.kFrontLeftTurningMotorPort,
		Constants.kFrontLeftDriveEncoderReversed,
		Constants.kFrontLeftTurningEncoderReversed,
		Constants.kFrontLeftDriveAbsoluteEncoderPort,
		Constants.kFrontLeftDriveAbsoluteEncoderOffsetDegrees,
		Constants.kFrontLeftDriveAbsoluteEncoderReversed);
	private final SwerveModule frontRight = new SwerveModule(
		Constants.kFrontRightDriveMotorPort,
		Constants.kFrontRightTurningMotorPort,
		Constants.kFrontRightDriveEncoderReversed,
		Constants.kFrontRightTurningEncoderReversed,
		Constants.kFrontRightDriveAbsoluteEncoderPort,
		Constants.kFrontRightDriveAbsoluteEncoderOffsetDegrees,
		Constants.kFrontRightDriveAbsoluteEncoderReversed);
	private final SwerveModule backLeft = new SwerveModule(
		Constants.kBackLeftDriveMotorPort,
		Constants.kBackLeftTurningMotorPort,
		Constants.kBackLeftDriveEncoderReversed,
		Constants.kBackLeftTurningEncoderReversed,
		Constants.kBackLeftDriveAbsoluteEncoderPort,
		Constants.kBackLeftDriveAbsoluteEncoderOffsetDegrees,
		Constants.kBackLeftDriveAbsoluteEncoderReversed);
	private final SwerveModule backRight = new SwerveModule(
		Constants.kBackRightDriveMotorPort,
		Constants.kBackRightTurningMotorPort,
		Constants.kBackRightDriveEncoderReversed,
		Constants.kBackRightTurningEncoderReversed,
		Constants.kBackRightDriveAbsoluteEncoderPort,
		Constants.kBackRightDriveAbsoluteEncoderOffsetDegrees,
		Constants.kBackRightDriveAbsoluteEncoderReversed);

	private static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Constants.kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, Constants.kMAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

	private static final SwerveDriveKinematics kDriveKinematics =
		new SwerveDriveKinematics(new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), 
								   new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), 
								   new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), 
								   new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

	/** Instance Variables ****************************************************/
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(this));

	ADIS16470_IMU gyroscope = new ADIS16470_IMU(); // The gyroscope sensor
	SwerveDriveOdometry odometry = new SwerveDriveOdometry(kDriveKinematics, Rotation2d.fromDegrees(gyroscope.getAngle()), new SwerveModulePosition[] {frontLeft.get, });

	/** Drivetrain Constructor *************************************************/
	public Drivetrain() {
		log.add("Constructor", LOG_LEVEL);
		zeroGyro();
	}
	/** initDefaultCommand ****************************************************
	 * Set the default command for the subsystem. */
	public void initDefaultCommand() {
		setDefaultCommand(null);
	}

	/** Gyroscope Methods ******************************************************/
  	public void zeroGyro() { // Zeroes the heading of the robot
    	gyroscope.reset();
	}
	public double getGyroAngle() { // Returns the heading of the robot
		return Math.IEEEremainder(gyroscope.getAngle(), 360); // Force the value between -180 and 180
	}

	/** Odometry Methods *******************************************************/
	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(getGyroAngle());
	}
	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(pose, this.getRotation2d());
	}
	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}
	public SwerveDriveKinematics getkDriveKinematics() {
		return kDriveKinematics;	
	}
	public TrapezoidProfile.Constraints getkThetaControllerConstraints() {
		return kThetaControllerConstraints;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Gyro Angle", getGyroAngle()); // The current gyroscope angle
		odometry.update(this.getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
	}

	// Stop all 4 swerve modules
	public void stopModules() {
		frontLeft.stop();
		frontRight.stop();
		backLeft.stop();
		backRight.stop();
	}
	// Update the current desired state of all 4 modules
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
		frontLeft.setDesiredState(desiredStates[0], false);
		frontRight.setDesiredState(desiredStates[1], false);
		backLeft.setDesiredState(desiredStates[2], false);
		backRight.setDesiredState(desiredStates[3], false);
	}
	// Drive the robot with the specified x, y, and rotation speeds
	public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldOriented) {

		ChassisSpeeds chassisSpeeds;

		if (fieldOriented) { // Field-Centric Driving
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getRotation2d());
		} else { // Robot-Centric Driving
			chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
		}

		SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
		setModuleStates(moduleStates);
	}

	// Provide access to the SwerveModule objects outside of this class
	public SwerveModule getFrontLeft() {
		return frontLeft;
	}
	public SwerveModule getFrontRight() {
		return frontRight;
	}
	public SwerveModule getBackLeft() {
		return backLeft;
	}
	public SwerveModule getBackRight() {
		return backRight;
	}
}