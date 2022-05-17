package utilities;

/**
 * A PID controller with settings for ranges on all values, and a velocity based
 * control mode.
 * 
 * @author Icarus Innovated
 */
public class ConfigurablePID
{
	/**
	 * The proportional component of the output. Equal to error * gain.
	 */
	private double proportional;

	/**
	 * The integral component of the output. Equal to itself plus error * gain.
	 */
	private double integral;

	/**
	 * The derivative component of the output. Equal to the rate of change of error
	 * * gain.
	 */
	private double derivative;

	/**
	 * The currently measured difference between the setpoint and process variable.
	 */
	private double currentError;

	/**
	 * The error from the previous run of the controller.
	 */
	private double pastError;

	/**
	 * The difference between the current and past error.
	 */
	private double errorDelta;

	/**
	 * The range where the controller will output 0.
	 */
	private double errorTolerance;

	/**
	 * The previously input process variable
	 */
	private double pastProcessVariable;

	/**
	 * The difference between the current and past process variables.
	 */
	private double processVariableVelocity;

	/**
	 * The output of the controller. Equal to proportional + integral + derivative.
	 */
	private double output;

	/**
	 * The multiplier applied to the error, creating a target speed. If 0, target
	 * speed is ignored.
	 */
	private double speed;

	/**
	 * The multiplier for the proportional component of the output
	 */
	private double proportionalGain;

	/**
	 * The multiplier for the integral component of the output
	 */
	private double integralGain;

	/**
	 * The multiplier for the derivative component of the output
	 */
	private double derivativeGain;

	/**
	 * The minimum value of the proportional component
	 */
	private double minProportional;

	/**
	 * The maximum value of the proportional component
	 */
	private double maxProportional;

	/**
	 * The minimum value of the integral component
	 */
	private double minIntegral;

	/**
	 * The maximum value of the integral component
	 */
	private double maxIntegral;

	/**
	 * The minimum value of the derivative component
	 */
	private double minDerivative;

	/**
	 * The maximum value of the derivative component
	 */
	private double maxDerivative;

	/**
	 * The minimum value of the total output
	 */
	private double minOutput;

	/**
	 * The maximum value of the total output
	 */
	private double maxOutput;

	/**
	 * If the proportional component should be clamped.
	 * Automatically set when configuring limits.
	 */
	private boolean clampProportional;

	/**
	 * If the integral component should be clamped.
	 * Automatically set when configuring limits.
	 */
	private boolean clampIntegral;

	/**
	 * If the derivative component should be clamped.
	 * Automatically set when configuring limits.
	 */
	private boolean clampDerivative;

	/**
	 * If the output should be clamped.
	 * Automatically set when configuring limits.
	 */
	private boolean clampOutput;

	/**
	 * If the controller should run in velocity mode.
	 * Automatically set when configuring speed.
	 */
	private boolean velocityMode;

	/**
	 * Creates a new Configurable PID with default parameters
	 */
	public ConfigurablePID()
	{
	}

	/**
	 * Creates a new configurable PID controller with the set gains.
	 * 
	 * @param proportionalGain scales the proportional of the error
	 * @param integralGain     scales the integral of the error
	 * @param derivativeGain   scales the derivative of the error
	 */
	public ConfigurablePID(double proportionalGain, double integralGain, double derivativeGain)
	{
		this.proportionalGain = proportionalGain;
		this.integralGain = integralGain;
		this.derivativeGain = derivativeGain;
	}

	/**
	 * Creates a new configurable PID controller using a PIDConfiguration.
	 * 
	 * @param configuration A PIDConfiguration to use the settings from.
	 */
	public ConfigurablePID(PIDConfiguration configuration)
	{
		this.proportionalGain = configuration.proportionalGain;
		this.integralGain = configuration.integralGain;
		this.derivativeGain = configuration.derivativeGain;
        this.speed = configuration.speed;
        this.errorTolerance = configuration.errorTolerance;
        this.minProportional = configuration.minProportional;
        this.maxProportional = configuration.maxProportional;
        this.minIntegral = configuration.minIntegral;
        this.maxIntegral = configuration.maxIntegral;
        this.minDerivative = configuration.minDerivative;
        this.maxDerivative = configuration.maxDerivative;
        this.minOutput = configuration.minOutput;
        this.maxOutput = configuration.maxOutput;
		this.velocityMode = this.speed != 0;
		this.clampOutput = this.minOutput != 0 || this.maxOutput != 0;
		this.clampProportional = this.minProportional != 0 || this.maxProportional != 0;
		this.clampIntegral = this.minIntegral != 0 || this.maxIntegral != 0;
		this.clampDerivative = this.minDerivative != 0 || this.maxDerivative != 0;
	}

	/**
	 * Computes a new output based on the set point and process variable. If a speed
	 * has been set, the output will be based on speed.
	 * 
	 * @param setpoint        The value that the system should try match the process
	 *                        variable to.
	 * @param processVariable The measured value from the system that is being
	 *                        controlled.
	 * @return The computed output to correct the process variable to the setpoint.
	 */
	public double runPID(double setpoint, double processVariable)
	{
		if (velocityMode)
		{
			processVariableVelocity = processVariable - pastProcessVariable;
			pastProcessVariable = processVariable;
			currentError = (setpoint - processVariable) * speed;
			currentError -= processVariableVelocity;
		} else
		{
			currentError = setpoint - processVariable;
		}
		if (Math.abs(setpoint - processVariable) < errorTolerance)
		{
			resetValues();
			return 0;
		}
		errorDelta = currentError - pastError;
		pastError = currentError;

		proportional = currentError * proportionalGain;
		if (clampProportional)
		{
			proportional = clamp(proportional, minProportional, maxProportional);
		}

		integral += currentError * integralGain;
		if (clampIntegral)
		{
			integral = clamp(integral, minIntegral, maxIntegral);
		}

		derivative = errorDelta * derivativeGain;
		if (clampDerivative)
		{
			derivative = clamp(derivative, minDerivative, maxDerivative);
		}

		output = proportional + integral + derivative;
		if (clampOutput)
		{
			output = clamp(output, minOutput, maxOutput);
		}

		return output;
	}

	/**
	 * Set the speed used in runPID(). Once speed has been set to a non-zero number,
	 * the controller will use velocity in its calculations.
	 *
	 * @param newSpeed The speed to use in runPID(). This is in units per program
	 *                 run.
	 */
	public void setSpeed(double newSpeed)
	{
		speed = newSpeed;
		velocityMode = speed != 0;
	}

	/**
	 * Set the minimum allowed output for the controller.
	 *
	 * @param newMinOutput The minimum output of the controller
	 */
	public void setMinOutput(double newMinOutput)
	{
		minOutput = newMinOutput;
		clampOutput = minOutput != 0 || maxOutput != 0;
	}

	/**
	 * Set the maximum allowed output for the controller.
	 *
	 * @param newMaxOutput the maximum output of the controller
	 */
	public void setMaxOutput(double newMaxOutput)
	{
		maxOutput = newMaxOutput;
		clampOutput = minOutput != 0 || maxOutput != 0;
	}

	/**
	 * Set the allowed range of the output of the controller.
	 *
	 * @param newMinOutput the minimum output
	 * @param newMaxOutput the maximum output
	 */
	public void setOutputRange(double newMinOutput, double newMaxOutput)
	{
		minOutput = newMinOutput;
		maxOutput = newMaxOutput;
		clampOutput = minOutput != 0 || maxOutput != 0;
	}

	/**
	 * Set the minimum allowed proportional component for the controller.
	 *
	 * @param newMinProportional the minimum proportional component
	 */
	public void setMinProportional(double newMinProportional)
	{
		minProportional = newMinProportional;
		clampProportional = minProportional != 0 || maxProportional != 0;
	}

	/**
	 * Set the maximum allowed proportional component for the controller.
	 *
	 * @param newMaxProportional the maximum proportional component
	 */
	public void setMaxProportional(double newMaxProportional)
	{
		maxProportional = newMaxProportional;
		clampProportional = minProportional != 0 || maxProportional != 0;
	}

	/**
	 * Set the allowed range of the proportional component for the controller.
	 *
	 * @param newMinProportional the minimum proportional component
	 * @param newMaxProportional the maximum proportional component
	 */
	public void setProportionalRange(double newMinProportional, double newMaxProportional)
	{
		minProportional = newMinProportional;
		maxProportional = newMaxProportional;
		clampProportional = minProportional != 0 || maxProportional != 0;
	}

	/**
	 * Set the minimum allowed integral component for the controller.
	 *
	 * @param newMinIntegral the minimum integral component
	 */
	public void setMinIntegral(double newMinIntegral)
	{
		minIntegral = newMinIntegral;
		clampIntegral = minIntegral != 0 || maxIntegral != 0;
	}

	/**
	 * Set the maximum allowed integral component for the controller.
	 *
	 * @param newMaxIntegral the maximum integral component
	 */
	public void setMaxIntegral(double newMaxIntegral)
	{
		maxIntegral = newMaxIntegral;
		clampIntegral = minIntegral != 0 || maxIntegral != 0;
	}

	/**
	 * Set the allowed range of the integral component for the controller.
	 *
	 * @param newMinIntegral the minimum integral component
	 * @param newMaxIntegral the maximum integral component
	 */
	public void setIntegralRange(double newMinIntegral, double newMaxIntegral)
	{
		minIntegral = newMinIntegral;
		maxIntegral = newMaxIntegral;
		clampIntegral = minIntegral != 0 || maxIntegral != 0;
	}

	/**
	 * Set the minimum allowed derivative component for the controller.
	 *
	 * @param newMinDerivative the minimum derivative component
	 */
	public void setMinderivative(double newMinDerivative)
	{
		minDerivative = newMinDerivative;
		clampDerivative = minDerivative != 0 || maxDerivative != 0;
	}

	/**
	 * Set the maximum allowed derivative component for the controller.
	 *
	 * @param newMaxDerivative the maximum derivative component
	 */
	public void setMaxderivative(double newMaxDerivative)
	{
		maxDerivative = newMaxDerivative;
		clampDerivative = minDerivative != 0 || maxDerivative != 0;
	}

	/**
	 * Set the allowed range of the derivative component for the controller.
	 *
	 * @param newMinDerivative the minimum derivative component
	 * @param newMaxDerivative the maximum derivative component
	 */
	public void setDerivativeRange(double newMinDerivative, double newMaxDerivative)
	{
		minDerivative = newMinDerivative;
		maxDerivative = newMaxDerivative;
		clampDerivative = minDerivative != 0 || maxDerivative != 0;
	}

	/**
	 * Set the proportional gain of the controller.
	 *
	 * @param newProportionalGain the proportional gain
	 */
	public void setProportionalGain(double newProportionalGain)
	{
		proportionalGain = newProportionalGain;
	}

	/**
	 * Get the total integral of the controller.
	 *
	 * @return integral
	 */
	public double getIntegral()
	{
		return integral;
	}

	/**
	 * Set the integral gain of the controller.
	 *
	 * @param newIntegralGain the integral gain
	 */
	public void setIntegralGain(double newIntegralGain)
	{
		integralGain = newIntegralGain;
	}

	/**
	 * Set the derivative gain of the controller.
	 *
	 * @param newDerivativeGain the derivative gain
	 */
	public void setderivativeGain(double newDerivativeGain)
	{
		derivativeGain = newDerivativeGain;
	}

	/**
	 * Get the last computed error value
	 * 
	 * @return last computed error
	 */
	public double getError()
	{
		return currentError;
	}

	/**
	 * Set the allowed error of the controller.
	 * 
	 * @param newErrorTolerance When the absolute value of the error is less than
	 *                          this number, the controller will output 0 and reset.
	 */
	public void setErrorTolerance(double newErrorTolerance)
	{
		errorTolerance = newErrorTolerance;
	}

	/**
	 * Reset values to 0. This function should be called whenever the controller
	 * stops running. This function will be called automatically when running the
	 * controller if a tolerance has been configured.
	 */
	public void resetValues()
	{
		proportional = 0;
		integral = 0;
		derivative = 0;
		currentError = 0;
		pastError = 0;
		processVariableVelocity = 0;
	}

	/**
	 * Limits the input value to the range of min to max.
	 * 
	 * @param value The value to be clamped.
	 * @param min   The smallest value to return.
	 * @param max   The largest value to return.
	 * @return      The value, within min and max.
	 */
	private double clamp(double value, double min, double max)
	{
		return Math.min(Math.max(value, min), max);
	}
}