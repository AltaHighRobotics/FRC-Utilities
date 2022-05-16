package utilities;

/**
 * A system for tracking robot position, and navigating to locations on the
 * field.
 * 
 * @author Icarus Innovated
 */
public class CopyPastAutonomous
{
	/**
	 * The computed drive power needed to reach the waypoint.
	 */
	private double drivePower;

	/**
	 * The computed steering power needed to aim at the waypoint.
	 */
	private double steeringPower;

	/**
	 * The current direction of the robot, in radians.
	 */
	private double heading;

	/**
	 * The direction the robot should attempt to face.
	 */
	private double targetHeading;

	/**
	 * The wrapped target heading, which represents the shortest path to the target.
	 */
	private double headingWrap;

	/**
	 * Controls if the robot drives in reverse or not.
	 */
	private boolean reverse;

	/**
	 * The position of the robot in 2D space.
	 */
	private CartesianVector position;

	/**
	 * The speed of the robot in 2D space.
	 */
	private CartesianVector velocity;

	/**
	 * The robot's target in 2D space.
	 */
	private CartesianVector target;

	/**
	 * The difference between the robot's target and its position.
	 */
	private CartesianVector positionError;

	/**
	 * The direction the waypoint is in from the current position.
	 */
	private double directionToWaypoint;

	/**
	 * The wrapped direction, which represents the shortest direction to steer.
	 */
	private double directionWrap;

	/**
	 * The speed the robot will attempt to drive at, based on the distance to the
	 * target.
	 */
	private double targetSpeed;

	/**
	 * The current position of the left and right encoders.
	 */
	private CartesianVector currentMotorPositions;

	/**
	 * The previous position of the left and right encoders.
	 */
	private CartesianVector previousMotorPositions;

	/**
	 * The difference between the current and previous encoder positions.
	 */
	private CartesianVector motorVelocities;

	/**
	 * The ratio between drive encoder units and full rotations of the drive motor.
	 * For example, a falcon 500 integrated encoder has 2048 steps per rotation,
	 * so for drivetrains running falcon 500s, set this to 2048.
	 */
	private final int ENCODER_UNITS_PER_ROTATION;

	/**
	 * The ratio between full rotations of the drive motor and inches travled.
	 * For example, for 4 inch wheel that is direct driven by the drive motor,
	 * the correct value would be PI * 4.
	 */
	private final double INCHES_PER_ROTATION;

	/**
	 * A PID controller for computing the steering power.
	 */
	private final ConfigurablePID drivetrainHeadingPID;

	/**
	 * A PID controller for computing the drive power.
	 */
	private final ConfigurablePID drivetrainSpeedPID;

	/**
	 * Creates a new CopyPastAutonomous with the set ratio constants and PID configurations.
	 * 
	 * @param encoderUnitsPerRotation The ratio between drive encoder units and full rotations of the drive motor.
	 * @param inchesPerRotation       The ratio between full rotations of the drive motor and inches travled.
	 * @param headingPIDConfig        A PIDConfiguration with settings for the heading controller.
	 * @param speedPIDConfig          A PIDConfiguration with settings for the speed controller.
	 */
	public CopyPastAutonomous(int encoderUnitsPerRotation, double inchesPerRotation, PIDConfiguration headingPIDConfig, PIDConfiguration speedPIDConfig)
	{
		this.position = new CartesianVector(0, 0);
		this.velocity = new CartesianVector(0, 0);
		this.target = new CartesianVector(0, 0);
		this.positionError = new CartesianVector(0, 0);

		this.currentMotorPositions = new CartesianVector(0, 0);
		this.previousMotorPositions = new CartesianVector(0, 0);
		this.motorVelocities = new CartesianVector(0, 0);

		this.ENCODER_UNITS_PER_ROTATION = encoderUnitsPerRotation;
		this.INCHES_PER_ROTATION = inchesPerRotation;

		this.drivetrainHeadingPID = new ConfigurablePID(headingPIDConfig);
		this.drivetrainSpeedPID = new ConfigurablePID(speedPIDConfig);
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

		currentMotorPositions.set(leftMotorEncoderPos / ENCODER_UNITS_PER_ROTATION,
				rightMotorEncoderPos / ENCODER_UNITS_PER_ROTATION);
		motorVelocities = currentMotorPositions.getSubtraction(previousMotorPositions);
		previousMotorPositions.copy(currentMotorPositions);
		motorVelocities.multiply(INCHES_PER_ROTATION);
		motorVelocities.average();

		velocity.set((Math.cos(heading) * motorVelocities.average), (Math.sin(heading) * motorVelocities.average));
		position.add(velocity);
	}

	/**
	 * Updates the internal target heading and target speed of the robot. The result
	 * is based on the currently set waypoint. Calling this function will also
	 * update the desired motor power levels.
	 * 
	 */
	public void updateTargetHeadingAndSpeed()
	{
		positionError = target.getSubtraction(position);

		directionToWaypoint = Math.atan2(positionError.y, positionError.x);
		directionWrap = (((directionToWaypoint - heading - Math.PI) % (Math.PI * 2)) + Math.PI);

		targetSpeed = Math.cos(directionWrap) * positionError.magnitude();
		drivePower = drivetrainSpeedPID.runPID(targetSpeed, motorVelocities.average);

		targetHeading = directionToWaypoint;
		if (reverse)
		{
			targetHeading += Math.PI;
		}

		headingWrap = (((targetHeading - heading - Math.PI) % (Math.PI * 2)) + Math.PI);
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
	public void setPosition(CartesianVector newPosition)
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
	public CartesianVector getPosition()
	{
		return position.clone();
	}

	/**
	 * Sets the reverse setting of the robot. This value controls if the robot will
	 * go to waypoints while driving backwards or forwards.
	 * 
	 * @param driveReverse A boolean for if the robot will drive in reverse. When
	 *                     true, it will drive backwards. When false, it will drive
	 *                     forwards.
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
	public void setWaypoint(final CartesianVector waypoint)
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
		return Math.abs(positionError.magnitude()) < UtilitiyConstants.MAX_WAYPOINT_ERROR;
	}

	/**
	 * A simple check for if the robot is within the configured MAX_WAYPOINT_ERROR
	 * of the input point.
	 * 
	 * @param point A 2D vector containing the x and y values to check against.
	 * @return True if the robot position is near the input point.
	 */
	public boolean isAtPoint(CartesianVector point)
	{
		return Math.abs(position.getSubtraction(point).magnitude()) < UtilitiyConstants.MAX_WAYPOINT_ERROR;
	}

}
