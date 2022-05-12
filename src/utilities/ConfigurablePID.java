package utilities;

/**
 * A PID controller with settings for ranges on all values, and a velocity based control mode.
 * 
 * @author Ian
 */
public class ConfigurablePID
{
	// The proportional component of the output. Equal to error * gain.
	private double proportional;

	// The integral component of the output. Equal to itself plus error * gain.
	private double integral;

	// The derivative component of the output. Equal to the rate of change of error * gain.
	private double derivative;

	// The currently measured difference between the setpoint and process variable.
	private double currentError;

	// The error from the previous run of the controller.
	private double pastError;

	// The difference between the current and past error.
	private double errorDelta;

	// The range where the controller will output 0.
	private double errorTolerance;

	// The previously input process variable
	private double pastProcessVariable;

	// The difference between the current and past process variables.
	private double processVariableVelocity;
	
	// The output of the controller. Equal to proportional + integral + derivative.
	private double output;

	// The multipler applied to the error, creating a target speed. If 0, target speed is ignored.
	private double speed;

	// The multiplier for the proportional component of the output
	private double proportionalGain;

	// The multiplier for the integral component of the output
	private double integralGain;

	// The multiplier for the derivative component of the output
	private double derivativeGain;

	// The minimum value of the proportional component
	private double minProportional;

	// The maximum value of the proportional component
	private double maxProportional;

	// The minimum value of the integral component
	private double minIntegral;

	// The maximum value of the integral component
	private double maxIntegral;

	// The minimum value of the derivative component
	private double minDerivative;

	// The maximum value of the derivative component
	private double maxDerivative;

	// The minimum value of the total output
	private double minOutput;

	// The maximum value of the total output
	private double maxOutput;

	public ConfigurablePID() {}

	/**
	 * Creates a new configurable PID controller with the set gains.
	 * 
	 * @param proportionalGain scales the proportional of the error
	 * @param integralGain scales the integral of the error
	 * @param derivativeGain scales the derivative of the error
	 */
	public ConfigurablePID(double proportionalGain, double integralGain, double derivativeGain)
	{
		this.proportionalGain = proportionalGain;
		this.integralGain = integralGain;
		this.derivativeGain = derivativeGain;
	}

	/**
	 * Computes a new output based on the setpoint and process variable. If a speed has been set, the output will be based on speed.
	 * 
	 * @param setpoint The value that the system should try match the process variable to.
	 * @param processVariable The measured value from the system that is being controlled.
	 * @return The computed output to correct the process variable to the setpoint.
	 */
	public double runPID(double setpoint, double processVariable)
	{
		if (speed != 0)
		{
			processVariableVelocity = processVariable - pastProcessVariable;
			pastProcessVariable = processVariable;
			currentError = (setpoint - processVariable) * speed;
			currentError -= processVariableVelocity;
		}
		else
		{
			currentError = setpoint - processVariable;
		}
		if (Math.abs(currentError) < errorTolerance) {
			resetValues();
			return 0;
		}
		errorDelta = currentError - pastError;
		pastError = currentError;

		proportional = currentError * proportionalGain;
		if (minProportional != 0 || maxProportional != 0)
		{
			proportional = clamp(proportional, minProportional, maxProportional);
		}

		integral += currentError * integralGain;
		if (minIntegral != 0 || maxIntegral != 0)
		{
			integral = clamp(integral, minIntegral, maxIntegral);
		}

		derivative = errorDelta * derivativeGain;
		if (minDerivative != 0 || maxDerivative != 0)
		{
			derivative = clamp(derivative, minDerivative, maxDerivative);
		}

		output = proportional + integral + derivative;
		if (minOutput != 0 || maxOutput != 0)
		{
			output = clamp(output, minOutput, maxOutput);
		}

		return output;
	}

	/**
	 * Set the speed used in runPID().
	 * Once speed has been set to a non-zero number, the controller will use velocity in its calculations.
	 *
	 * @param newSpeed The speed to use in runPID(). This is in units per program run.
	 */
	public void setSpeed(double newSpeed)
	{
		speed = newSpeed;
	}

	/**
	 * Set the minimum allowed output for the controller.
	 *
	 * @param newMinOutput The minimum output of the controller
	 */
	public void setMinOutput(double newMinOutput)
	{
		minOutput = newMinOutput;
	}

	/**
	 * Set the maximum allowed output for the controller.
	 *
	 * @param newMaxOutput the maximum output of the controller
	 */
	public void setMaxOutput(double newMaxOutput)
	{
		maxOutput = newMaxOutput;
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
	}

	/**
	 * Set the minimum allowed proportional component for the controller.
	 *
	 * @param newMinProportional the minimum proportional component
	 */
	public void setMinProportional(double newMinProportional)
	{
		minProportional = newMinProportional;
	}

	/**
	 * Set the maximum allowed proportional component for the controller.
	 *
	 * @param newMaxProportional the maximum proportional component
	 */
	public void setMaxProportional(double newMaxProportional)
	{
		maxProportional = newMaxProportional;
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
	}

	/**
	 * Set the minimum allowed integral component for the controller.
	 *
	 * @param newMinIntegral the minimum integral component
	 */
	public void setMinIntegral(double newMinIntegral)
	{
		minIntegral = newMinIntegral;
	}

	/**
	 * Set the maximum allowed integral component for the controller.
	 *
	 * @param newMaxIntegral the maximum integral component
	 */
	public void setMaxIntegral(double newMaxIntegral)
	{
		maxIntegral = newMaxIntegral;
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
	}

	/**
	 * Set the minimum allowed derivative component for the controller.
	 *
	 * @param newMinDerivative the minimum derivative component
	 */
	public void setMinderivative(double newMinDerivative)
	{
		minDerivative = newMinDerivative;
	}

	/**
	 * Set the maximum allowed derivative component for the controller.
	 *
	 * @param newMaxDerivative the maximum derivative component
	 */
	public void setMaxderivative(double newMaxDerivative)
	{
		maxDerivative = newMaxDerivative;
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
	 * @param newErrorTolerance When the absolute value of the error is less than this number, 
	 * the controller will output 0 and reset.
	 */
	public void setErrorTolerance(double newErrorTolerance)
	{
		errorTolerance = newErrorTolerance;
	}

	/**
	 * Reset values to 0. This function should be called whenever the controller stops running.
	 * This function will be called automatically when running the controller if a tolerance has been configured.
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

	private double clamp(double value, double min, double max)
	{
		return Math.min(Math.max(value, min), max);
	}
}
