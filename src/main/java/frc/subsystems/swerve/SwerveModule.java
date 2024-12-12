package frc.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.Constants;

public class SwerveModule {

    private static final double kDriveEncoderRot2Meter = Constants.kDriveEncoderRot2Meter;
    private static final double kDriveEncoderRPM2MeterPerSec = Constants.kDriveEncoderRPM2MeterPerSec;
    private static final double kTurningEncoderRot2Rad = Constants.kTurningEncoderRot2Rad;
    private static final double kTurningEncoderRPM2RadPerSec = Constants.kTurningEncoderRPM2RadPerSec;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetDegrees;

    // Construct a new "SwerveModule" object (this method is the constructor)
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetDegrees = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);
        //TODO: possible problem area- do not trust this code!!!
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
        // more configurations
        absoluteEncoder.getConfigurator().apply(config);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveMotor.enableVoltageCompensation(Constants.nominalVoltage);
        turningMotor.enableVoltageCompensation(Constants.nominalVoltage);

        driveMotor.setSmartCurrentLimit(Constants.driveCurrentLimit);
        turningMotor.setSmartCurrentLimit(Constants.steerCurrentLimit);

        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        
        driveEncoder.setPositionConversionFactor(kDriveEncoderRot2Meter); 
        driveEncoder.setVelocityConversionFactor(kDriveEncoderRPM2MeterPerSec); 
        turningEncoder.setPositionConversionFactor(kTurningEncoderRot2Rad); 
        turningEncoder.setVelocityConversionFactor(kTurningEncoderRPM2RadPerSec); 
   
        turningPidController = new PIDController(Constants.kP_Turning, 0, 0); // There is no need for an I term or D term, the P term is enough on its own! :)
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    // Methods for getting the current velocity/position of the motors
    public double getDrivePosition() {
        return driveEncoder.getPosition(); // configured to meters
    }
    public double getTurningPosition() {
        return turningEncoder.getPosition(); // configured to radians
    }
    public double getDriveVelocity() {
        return driveEncoder.getVelocity(); // configured to meters per second
    }
    public double getTurningVelocity() {
        return turningEncoder.getVelocity(); // configured to radians per second
    }

    // Get the absolute position of the module in radians
    public double getAbsoluteEncoderRadians() {
        //TODO: May need to change the getAbsolutePosition to getRotorPosition. What's the difference? idk
        double angle = absoluteEncoder.getAbsolutePosition().getValue(); // Gets the absolute position, ranging from -0.5 to 1
        angle *= (Math.PI / 180); // Convert degrees to radians
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0); // Multiply by -1 if the encoder is reversed
    }

    // Reset the encoders to their starting positions
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRadians());
    }

    // Get the current SwerveModuleState of the module
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public static double neoToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / (gearRatio * 2048.0));
        //TODO: after figuring out what gearratio is, create getPosition() method to fix the SwerveDriveOdometry object declaration in Drivetrain.java
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(neoToMeters(driveEncoder.getPosition(), Constants.kWheelCircumferenceMeters,));

    }


    // Set the currently desired SwerveModuleState
    public void setDesiredState(SwerveModuleState state, boolean xStance) {
        state = SwerveModuleState.optimize(state, getState().angle);

        if (!xStance && Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }

        driveMotor.setVoltage(state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond * Constants.nominalVoltage);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    // Set 0% power to both motors
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
