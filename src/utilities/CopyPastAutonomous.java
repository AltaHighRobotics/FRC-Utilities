package utilities;

public class CopyPastAutonomous
{
	private final ConfigurablePID drivetrainHeadingPID;
	private final ConfigurablePID drivetrainSpeedPID;

	private double drivePower;
	private double steeringPower;

	private double currentHeading;
	private double previousHeading;
	private double headingError;
	private double headingRate;

	private vector currentMotorPositions;
	private vector previousMotorPositions;
	private vector motorVelocities;
	private vector velocity;
	private vector position;
	private vector positionError;

	private vector target;
	private double targetHeading;
	private double targetSpeed;

	public CopyPastAutonomous(ConfigurablePID[] PIDArray)
	{
		this.drivePower = 0;
		this.steeringPower = 0;
		this.targetHeading = 0;
		this.position = new vector(0, 0);
		this.velocity = new vector(0, 0);
		this.target = new vector(0, 0);
		this.positionError = new vector(0, 0);
		this.currentMotorPositions = new vector(0, 0);
		this.previousMotorPositions = new vector(0, 0);
		this.motorVelocities = new vector(0, 0);
		this.targetSpeed = 0;

		this.drivetrainHeadingPID = PIDArray[0];
		this.drivetrainSpeedPID = PIDArray[1];
	}

	/**
	 * Updates where the robot believes itself to be. This function should be called
	 * before any others, and also resets power levels as a safety.
	 * 
	 * @param leftMotorEncoderPos  The raw encoder value of the left side of the
	 *                             robot.
	 * @param rightMotorEncoderPos The raw encoder value of the right side of the
	 *                             robot.
	 * @param yaw                  The compass heading of the robot, in degrees, as
	 *                             measured by an IMU.
	 */
	public void updateRobotPositon(double leftMotorEncoderPos, double rightMotorEncoderPos, double yaw)
	{
		drivePower = 0;
		steeringPower = 0;
		currentHeading = Math.toRadians(yaw);
		headingRate = currentHeading - previousHeading;
		previousHeading = currentHeading;

		currentMotorPositions.set(leftMotorEncoderPos / PastaConstants.ENCODER_UNITS_PER_ROTATION,
				rightMotorEncoderPos / PastaConstants.ENCODER_UNITS_PER_ROTATION);
		motorVelocities = currentMotorPositions.getSubtraction(previousMotorPositions);
		previousMotorPositions.copy(currentMotorPositions);
		motorVelocities.multiply(PastaConstants.INCHES_PER_ROTATION);
		motorVelocities.average();

		velocity.set((Math.cos(currentHeading) * motorVelocities.average),
				(Math.sin(currentHeading) * motorVelocities.average));
		position.add(velocity);
	}

	/**
	 * Updates the internal target heading and target speed of the robot. The result
	 * is based on the currently set waypoint.
	 * 
	 */
	private void updateTargetHeadingAndSpeed()
	{
		positionError = target.getSubtraction(position);
		targetHeading = Math.atan2(positionError.y, positionError.x);
		headingError = targetHeading - currentHeading;
		targetSpeed = Math.cos(headingError) * positionError.magnitude();
	}

	/**
	 * Updates the steering and drive throttle levels using the target waypoint.
	 * 
	 */
	public void updateDriveAndSteeringPower()
	{
		updateTargetHeadingAndSpeed();
		steeringPower = drivetrainHeadingPID.runVelocityPID(targetHeading, currentHeading, headingRate);
		drivePower = drivetrainSpeedPID.runPID(targetSpeed, motorVelocities.average);
	}

	/**
	 * Updates the steering and drive throttle levels using the target waypoint.
	 * 
	 * @param invertHeading If this is set to true, the robot will drive backwards
	 *                      to the waypoint.
	 */
	public void updateDriveAndSteeringPower(boolean invertHeading)
	{
		updateTargetHeadingAndSpeed();
		if (invertHeading)
		{
			steeringPower = drivetrainHeadingPID.runVelocityPID(targetHeading + Math.PI, currentHeading, headingRate);
		} else
		{
			steeringPower = drivetrainHeadingPID.runVelocityPID(targetHeading, currentHeading, headingRate);
		}
		drivePower = drivetrainSpeedPID.runPID(targetSpeed, motorVelocities.average);
	}

	/**
	 * Gets the desired steering power of the robot, to reach the target waypoint,
	 * as of the last update.
	 * 
	 * @return The power level, from -1 to 1, that the robot needs the steering to
	 *         be set to.
	 */
	public double getSteeringPower()
	{
		return steeringPower;
	}

	/**
	 * Gets the desired drive power of the robot, to reach the target waypoint, as
	 * of the last update.
	 * 
	 * @return The power level, from -1 to 1, that the robot needs the drive to be
	 *         set to.
	 */
	public double getDrivePower()
	{
		return drivePower;
	}

	/**
	 * Sets the robots current position to a new value, erasing any previous
	 * position tracking.
	 * 
	 * @param newPosition A 2D vector object, containing the desired x and y values
	 *                    to set the position to.
	 */
	public void setPosition(vector newPosition)
	{
		position.copy(newPosition);
	}

	/**
	 * Gets the currently tracked position of the robot. This is updated whenever
	 * updatePosition() or setPosition() are called.
	 * 
	 * @return The tracked position of the robot. If updatePosition() has been
	 *         running, this will reflect the robots position, in inches.
	 */
	public vector getPosition()
	{
		return position.clone();
	}

	/**
	 * Sets the robots current target, which is used by
	 * updateTargetHeadingAndSpeed().
	 * 
	 * @param waypoint A 2D vector object, containing the desired x and y values to
	 *                 set the target waypoint to.
	 */
	public void setWaypoint(final vector waypoint)
	{
		target.copy(waypoint);
	}

	/**
	 * A simple check for if the robot is within the configured MAX_WAYPOINT_ERROR
	 * of the currently set waypoint.
	 * 
	 * @return True if the robot position is near the waypoint.
	 */
	public boolean hasReachedWaypoint()
	{
		return Math.abs(positionError.magnitude()) < PastaConstants.MAX_WAYPOINT_ERROR;
	}

	/**
	 * A simple check for if the robot is within the configured MAX_WAYPOINT_ERROR
	 * of the input point.
	 * 
	 * @param point A 2D vector containing the x and y values to check against.
	 * @return True if the robot position is near the input point.
	 */
	public boolean isAtPoint(vector point)
	{
		return Math.abs(position.getSubtraction(point).magnitude()) < PastaConstants.MAX_WAYPOINT_ERROR;
	}

}
