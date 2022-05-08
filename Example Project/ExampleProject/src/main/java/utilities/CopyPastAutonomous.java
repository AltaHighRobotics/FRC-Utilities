package utilities;



public class CopyPastAutonomous
{
	private final ConfigurablePID drivetrainHeadingPID;
	private final ConfigurablePID drivetrainSpeedPID;

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

	public CopyPastAutonomous(ConfigurablePID[] PIDArray)
	{
		this.robotY = 0;
		this.robotX = 0;
		this.targetX = 0;
		this.targetY = 0;
		this.targetHeading = 0;

		this.drivetrainHeadingPID = PIDArray[0];
		this.drivetrainSpeedPID = PIDArray[1];
	}

	public void setArcadeDrive(double forward, double turn)
	{

	}

	public void setSwerveDrive(double forward, double turn)
	{

	}

	public void drivetrainPositionIntegration(double leftMotorEncoderPos, double rightMotorEncodePos, double yaw)
	{
		currentLeftMotorPosition = leftMotorEncoderPos / PastaConstants.ENCODER_ROTATION_UNITS;
		currentRightMotorPosition = rightMotorEncodePos / PastaConstants.ENCODER_ROTATION_UNITS;

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
	 * DriveTrain drives to the specified waypoint No longer functions, uses methods
	 * no longer supported
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
		// this.currentHeading = (double) this.navX.getYaw();
		this.headingRate = this.currentHeading - this.previousHeading;
		this.headingError = this.targetHeading - this.currentHeading;
		this.previousHeading = this.currentHeading;

		this.distanceError = Math
				.sqrt(Math.pow(this.targetY - this.robotY, 2) + Math.pow(this.targetX - this.robotX, 2));

		this.steeringPower = this.drivetrainHeadingPID.runVelocityPID(this.targetHeading, this.currentHeading,
				this.headingRate);

		if (Math.abs(this.headingError) < PastaConstants.MAX_DRIVE_HEADING_ERROR)
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
			// stopMotors();
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
		return Math.abs(this.distanceError) < PastaConstants.MAX_WAYPOINT_ERROR;
	}

	public void setPos(double x, double y)
	{
		this.robotX = x;
		this.robotY = y;
	}
}
