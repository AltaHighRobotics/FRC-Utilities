package utilities;

import utilities.PastaConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;

public class CopyPastautonomous
{
	private final SupplyCurrentLimitConfiguration drivetrainCurrentLimit;

	private final WPI_TalonFX rightMotorFront;
	private final WPI_TalonFX rightMotorBack;
	private final WPI_TalonFX leftMotorFront;
	private final WPI_TalonFX leftMotorBack;

	private final ConfigurablePID drivetrainHeadingPID;
	private final ConfigurablePID drivetrainSpeedPID;

	private final AHRS navX;

	private double[] pos = { -9999, -9999 };

	private double currentRightMotorPosition;
	private double currentLeftMotorPosition;

	private double drivePower;
	private double steeringPower;

	private double currentHeading;
	private double previousHeading;
	private double headingError;
	private double headingRate;

	private double previousRightMotorPosition;
	private double previousLeftMotorPosition;
	private double distanceTraveledLeft;
	private double distanceTraveledRight;
	private double distanceTraveled;
	private double distanceError;
	private double robotX;
	private double robotY;

	private double targetX;
	private double targetY;
	private double targetHeading;

	public CopyPastautonomous(WPI_TalonFX[] driveMotors, ConfigurablePID[] PIDArray, AHRS controlBoard)
	{
		this.rightMotorFront = driveMotors[0];
		this.rightMotorBack = driveMotors[1];
		this.leftMotorFront = driveMotors[2];
		this.leftMotorBack = driveMotors[3];

		this.robotY = 0;
		this.robotX = 0;
		this.targetX = 0;
		this.targetY = 0;
		this.targetHeading = 0;

		this.drivetrainHeadingPID = PIDArray[0];

		this.drivetrainSpeedPID = PIDArray[1];

		this.navX = controlBoard;

		this.drivetrainCurrentLimit = new SupplyCurrentLimitConfiguration(true, Constants.DRIVETRAIN_CURRENT_LIMIT, 0,
				0);

		this.rightMotorFront.configFactoryDefault();
		this.rightMotorBack.configFactoryDefault();
		this.leftMotorFront.configFactoryDefault();
		this.rightMotorFront.configFactoryDefault();

		this.rightMotorFront.setSensorPhase(false);
		this.rightMotorBack.setSensorPhase(false);
		this.leftMotorFront.setSensorPhase(true);
		this.leftMotorBack.setSensorPhase(true);

		this.rightMotorFront.setInverted(TalonFXInvertType.Clockwise);
		this.rightMotorBack.setInverted(TalonFXInvertType.Clockwise);
		this.leftMotorFront.setInverted(TalonFXInvertType.CounterClockwise);
		this.leftMotorBack.setInverted(TalonFXInvertType.CounterClockwise);

		this.rightMotorFront.setNeutralMode(NeutralMode.Brake);
		this.rightMotorBack.setNeutralMode(NeutralMode.Brake);
		this.leftMotorFront.setNeutralMode(NeutralMode.Brake);
		this.leftMotorBack.setNeutralMode(NeutralMode.Brake);

		this.rightMotorFront.configOpenloopRamp(Constants.DRIVETRAIN_POWER_RAMP_TIME, 0);
		this.rightMotorBack.configOpenloopRamp(Constants.DRIVETRAIN_POWER_RAMP_TIME, 0);
		this.leftMotorFront.configOpenloopRamp(Constants.DRIVETRAIN_POWER_RAMP_TIME, 0);
		this.rightMotorFront.configOpenloopRamp(Constants.DRIVETRAIN_POWER_RAMP_TIME, 0);

		this.rightMotorFront.configSupplyCurrentLimit(drivetrainCurrentLimit);
		this.rightMotorBack.configSupplyCurrentLimit(drivetrainCurrentLimit);
		this.leftMotorFront.configSupplyCurrentLimit(drivetrainCurrentLimit);
		this.rightMotorFront.configSupplyCurrentLimit(drivetrainCurrentLimit);

		this.rightMotorBack.follow(this.rightMotorFront);
		this.leftMotorBack.follow(this.leftMotorFront);

		this.rightMotorBack.setStatusFramePeriod(1, 255);
		this.rightMotorBack.setStatusFramePeriod(2, 255);
		this.leftMotorBack.setStatusFramePeriod(1, 255);
		this.leftMotorBack.setStatusFramePeriod(2, 255);
	}

	public void setArcadeDrive(double forward, double turn)
	{

	}

	public void setSwerveDrive(double forward, double turn)
	{

	}

	public void drivetrainPositionIntegration(double leftMotorEncoderPos, double rightMotorEncodePos, double yaw)
	{
		currentLeftMotorPosition = leftMotorFront.getSelectedSensorPosition() / PastaConstants.ENCODER_ROTATION_UNITS;
		currentRightMotorPosition = rightMotorFront.getSelectedSensorPosition() / PastaConstants.ENCODER_ROTATION_UNITS;

		currentHeading = yaw;

		currentHeading = Math.toRadians(currentHeading);

		distanceTraveledLeft = PastaConstants.DRIVETRAIN_ROTATION_DISTANCE_RATIO * PastaConstants.DRIVETRAIN_GEAR_RATIO
				* (currentLeftMotorPosition - previousLeftMotorPosition);
		distanceTraveledRight = PastaConstants.DRIVETRAIN_ROTATION_DISTANCE_RATIO * PastaConstants.DRIVETRAIN_GEAR_RATIO
				* (currentRightMotorPosition - previousRightMotorPosition);
		distanceTraveled = (distanceTraveledLeft + distanceTraveledRight) / 2;

		previousLeftMotorPosition = currentLeftMotorPosition;
		previousRightMotorPosition = currentRightMotorPosition;

		robotX = robotX + (Math.cos(currentHeading) * distanceTraveled);
		robotY = robotY + (Math.sin(currentHeading) * distanceTraveled);
	}

	public void driveForwardTo(double waypointX, double waypointY)
	{

	}

	public void driveBackwardsTo(double waypointX, double waypointY)
	{

	}

	public double[] getPos()
	{
		pos[0] = robotX;
		pos[1] = robotY;
		return pos;
	}

	/**
	 * DriveTrain drives to wayPoint specified
	 * 
	 * @param waypointX
	 * @param waypointY
	 * @param driveBackwards
	 * @return if it has reached a waypoint
	 * @deprecated
	 */
	public boolean setDriveToWaypoint(double waypointX, double waypointY, boolean driveBackwards)
	{
		this.targetX = waypointX;
		this.targetY = waypointY;
		if (!driveBackwards)
		{
			this.targetHeading = Math.toDegrees(Math.atan2(this.targetY - this.robotY, this.targetX - this.robotX));
		} else
		{
			this.targetHeading = Math
					.toDegrees(Math.atan2(-(this.targetY - this.robotY), -(this.targetX - this.robotX)));
		}
		this.currentHeading = (double) this.navX.getYaw();
		this.headingRate = this.currentHeading - this.previousHeading;
		this.headingError = this.targetHeading - this.currentHeading;
		this.previousHeading = this.currentHeading;

		this.distanceError = Math
				.sqrt(Math.pow(this.targetY - this.robotY, 2) + Math.pow(this.targetX - this.robotX, 2));

		this.steeringPower = this.drivetrainHeadingPID.runVelocityPID(this.targetHeading, this.currentHeading,
				this.headingRate);

		if (Math.abs(this.headingError) < Constants.MAX_DRIVE_HEADING_ERROR)
		{
			this.drivePower = this.drivetrainSpeedPID.runPID(0, -this.distanceError);
			if (driveBackwards)
			{
				this.drivePower = -this.drivePower;
			}
		} else
		{
			this.drivePower = 0;
		}
		if (hasReachedWaypoint())
		{
			stopMotors();
		} else
		{
			this.setArcadeDrive(this.drivePower, this.steeringPower);
		}
		return hasReachedWaypoint();
	}

	public boolean pointAtWaypoint(double waypointX, double waypointY)
	{
		return false;

	}

	public void pointAtAngle(double targetAngle)
	{

	}

	public boolean hasReachedWaypoint()
	{
		return Math.abs(this.distanceError) < Constants.MAX_WAYPOINT_ERROR;
	}

	public void stopMotors()
	{
		this.leftMotorFront.neutralOutput();
		this.rightMotorFront.neutralOutput();
	}

	public void setPos(double x, double y)
	{
		this.robotX = x;
		this.robotY = y;
	}

	public void resetYaw()
	{
		this.navX.zeroYaw();
	}
}
