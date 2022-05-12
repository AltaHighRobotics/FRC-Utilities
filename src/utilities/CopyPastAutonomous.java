package utilities;

public class CopyPastAutonomous
{
	private double drivePower;
	private double steeringPower;

	private double heading;
	private double targetHeading;
	private double headingWrap;
	private boolean reverse;

	private vector position;
	private vector velocity;
	private vector target;
	private vector positionError;
	private double directionToWaypoint;
	private double directionWrap;
	private double targetSpeed;

	private vector currentMotorPositions;
	private vector previousMotorPositions;
	private vector motorVelocities;

	private final ConfigurablePID drivetrainHeadingPID;
	private final ConfigurablePID drivetrainSpeedPID;

	public CopyPastAutonomous(ConfigurablePID[] PIDArray)
	{
		this.drivePower = 0;
		this.steeringPower = 0;
		this.heading = 0;
		this.targetHeading = 0;
		this.headingWrap = 0;
		this.reverse = false;

		this.position = new vector(0, 0);
		this.velocity = new vector(0, 0);
		this.target = new vector(0, 0);
		this.positionError = new vector(0, 0);
		this.directionToWaypoint = 0;
		this.directionWrap = 0;
		this.targetSpeed = 0;

		this.currentMotorPositions = new vector(0, 0);
		this.previousMotorPositions = new vector(0, 0);
		this.motorVelocities = new vector(0, 0);

		this.drivetrainHeadingPID = new ConfigurablePID();
		this.drivetrainSpeedPID = new ConfigurablePID();
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
		heading = Math.toRadians(yaw);

		currentMotorPositions.set(leftMotorEncoderPos / PastaConstants.ENCODER_UNITS_PER_ROTATION,
				rightMotorEncoderPos / PastaConstants.ENCODER_UNITS_PER_ROTATION);
		motorVelocities = currentMotorPositions.getSubtraction(previousMotorPositions);
		previousMotorPositions.copy(currentMotorPositions);
		motorVelocities.multiply(PastaConstants.INCHES_PER_ROTATION);
		motorVelocities.average();

		velocity.set((Math.cos(heading) * motorVelocities.average),
				(Math.sin(heading) * motorVelocities.average));
		position.add(velocity);
	}

	/**
	 * Updates the internal target heading and target speed of the robot. The result
	 * is based on the currently set waypoint.
	 * Calling this function will also update the desired motor power levels.
	 * 
	 */
	public void updateTargetHeadingAndSpeed()
	{
		positionError = target.getSubtraction(position);

		directionToWaypoint = Math.atan2(positionError.y, positionError.x);
		directionWrap = (((directionToWaypoint - heading - Math.PI)%(Math.PI*2)) + Math.PI);

		targetSpeed = Math.cos(directionWrap) * positionError.magnitude();
		drivePower = drivetrainSpeedPID.runPID(targetSpeed, motorVelocities.average);

		targetHeading = directionToWaypoint;
		if (reverse)
		{
			targetHeading += Math.PI;
		}

		headingWrap = (((targetHeading - heading - Math.PI)%(Math.PI*2)) + Math.PI);
		steeringPower = drivetrainHeadingPID.runPID(headingWrap, 0);
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
	 * Sets the reverse setting of the robot. This value controls if the robot will go to waypoints while driving backwards or forwards.
	 * 
	 * @param driveReverse A boolean for if the robot will drive in reverse. When true, it will drive backwards. When false, it will drive forwards.
	 */
	public void setReverse(boolean driveReverse)
	{
		reverse = driveReverse;
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
