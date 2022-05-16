package utilities;

/**
 * A configuration for a ConfigurablePID. Includes all possible settings for ranges on all values.
 * 
 * @author Icarus Innovated
 */
public class PIDConfiguration
{
	/**
	 * The range where the controller will output 0.
	 */
	public double errorTolerance;

	/**
	 * The multiplier applied to the error, creating a target speed. If 0, target
	 * speed is ignored.
	 */
	public double speed;

	/**
	 * The multiplier for the proportional component of the output
	 */
	public double proportionalGain;

	/**
	 * The multiplier for the integral component of the output
	 */
	public double integralGain;

	/**
	 * The multiplier for the derivative component of the output
	 */
	public double derivativeGain;

	/**
	 * The minimum value of the proportional component
	 */
	public double minProportional;

	/**
	 * The maximum value of the proportional component
	 */
	public double maxProportional;

	/**
	 * The minimum value of the integral component
	 */
	public double minIntegral;

	/**
	 * The maximum value of the integral component
	 */
	public double maxIntegral;

	/**
	 * The minimum value of the derivative component
	 */
	public double minDerivative;

	/**
	 * The maximum value of the derivative component
	 */
	public double maxDerivative;

	/**
	 * The minimum value of the total output
	 */
	public double minOutput;

	/**
	 * The maximum value of the total output
	 */
	public double maxOutput;

    /**
     * Creates a new configuration.
     * 
     * @param proportionalGain
     * @param integralGain
     * @param derivativeGain
     * @param speed
     * @param errorTolerance
     * @param minProportional
     * @param maxProportional
     * @param minIntegral
     * @param maxIntegral
     * @param minDerivative
     * @param maxDerivative
     * @param minOutput
     * @param maxOutput
     */
    public PIDConfiguration(double proportionalGain, double integralGain, double derivativeGain, double speed,
            double errorTolerance, double minProportional, double maxProportional, double minIntegral,
            double maxIntegral, double minDerivative, double maxDerivative, double minOutput, double maxOutput)
    {
        this.proportionalGain = proportionalGain;
        this.integralGain = integralGain;
        this.derivativeGain = derivativeGain;
        this.speed = speed;
        this.errorTolerance = errorTolerance;
        this.minProportional = minProportional;
        this.maxProportional = maxProportional;
        this.minIntegral = minIntegral;
        this.maxIntegral = maxIntegral;
        this.minDerivative = minDerivative;
        this.maxDerivative = maxDerivative;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }
}