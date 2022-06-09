package utilities;

/**
 * A class for creating and working with 2D and 3D vectors Each instance
 * contains an x, y, and optionally a z value, along with various functions for
 * performing standard math operations on itself.
 * 
 * @author Icarus Innovated
 */
public class CartesianVector
{
	/**
	 * The x component of the vector
	 */
	public double x;

	/**
	 * The y component of the vector
	 */
	public double y;

	/**
	 * The z component of the vector. If a vector is created with only two
	 * parameters, this value will remain 0
	 */
	public double z;

	/**
	 * The a component of the vector. If a vector is created with only threee
	 * parameters, this value will remain 0
	 */
	public double a;

	/**
	 * The b component of the vector. If a vector is created with only four
	 * parameters, this value will remain 0
	 */
	public double b;

	/**
	 * The c component of the vector. If a vector is created with only five
	 * parameters, this value will remain 0
	 */
	public double c;

	/**
	 * The direction of the vector, in radians. This is 0 unless the direction
	 * function is called.
	 */
	public double direction;

	/**
	 * The magnitude of the vector. This is 0 unless the magnitude function is
	 * called.
	 */
	public double magnitude;

	/**
	 * The average value of the components of the vector. This is 0 unless the
	 * average function is called.
	 */
	public double average;

	/**
	 * The amount of dimensions the vector was created with.
	 */
	public final int dimensions;

	/**
	 * Makes a 2D vector object with the specified values.
	 * 
	 * @param initialX A double representing the initial x value.
	 * @param initialY A double representing the initial y value.
	 */
	public CartesianVector(double initialX, double initialY)
	{
		this.x = initialX;
		this.y = initialY;
		this.dimensions = 2;
	}

	/**
	 * Makes a 3D vector object with the specified values.
	 * 
	 * @param initialX A double representing the initial x value.
	 * @param initialY A double representing the initial y value.
	 * @param initialZ A double representing the initial z value.
	 */
	public CartesianVector(double initialX, double initialY, double initialZ)
	{
		this.x = initialX;
		this.y = initialY;
		this.z = initialZ;
		this.dimensions = 3;
	}

	/**
	 * Makes a 4D vector object with the specified values.
	 * 
	 * @param initialX A double representing the initial x value.
	 * @param initialY A double representing the initial y value.
	 * @param initialZ A double representing the initial z value.
	 * @param initialA A double representing the initial a value.
	 */
	public CartesianVector(double initialX, double initialY, double initialZ, double initialA)
	{
		this.x = initialX;
		this.y = initialY;
		this.z = initialZ;
		this.a = initialA;
		this.dimensions = 4;
	}

	/**
	 * Makes a 5D vector object with the specified values.
	 * 
	 * @param initialX A double representing the initial x value.
	 * @param initialY A double representing the initial y value.
	 * @param initialZ A double representing the initial z value.
	 * @param initialA A double representing the initial a value.
	 * @param initialB A double representing the initial b value.
	 */
	public CartesianVector(double initialX, double initialY, double initialZ, double initialA, double initialB)
	{
		this.x = initialX;
		this.y = initialY;
		this.z = initialZ;
		this.a = initialA;
		this.b = initialB;
		this.dimensions = 5;
	}

	/**
	 * Makes a 6D vector object with the specified values.
	 * 
	 * @param initialX A double representing the initial x value.
	 * @param initialY A double representing the initial y value.
	 * @param initialZ A double representing the initial z value.
	 * @param initialA A double representing the initial a value.
	 * @param initialB A double representing the initial b value.
	 * @param initialC A double representing the initial c value.
	 */
	public CartesianVector(double initialX, double initialY, double initialZ, double initialA, double initialB, double initialC)
	{
		this.x = initialX;
		this.y = initialY;
		this.z = initialZ;
		this.a = initialA;
		this.b = initialB;
		this.c = initialC;
		this.dimensions = 6;
	}
	
	/**
	 * Returns a vector object with the specified values and forces it to match the
	 * given type.
	 * 
	 * @param forceDimension The amount of dimensions the new vector should have.
	 * @param initialX A double representing the initial x value.
	 * @param initialY A double representing the initial y value.
	 * @param initialZ A double representing the initial z value.
	 * @param initialA A double representing the initial a value.
	 * @param initialB A double representing the initial b value.
	 * @param initialC A double representing the initial c value.
	 * @return A vector with the same values and dimensions as the inputs.
	 */
	private CartesianVector vectorType(int forceDimension, double initialX, double initialY, double initialZ, double initialA, double initialB, double initialC)
	{
		switch (forceDimension)
		{
			case 2:
				return new CartesianVector(initialX, initialY);
			case 3:
				return new CartesianVector(initialX, initialY, initialZ);
			case 4:
				return new CartesianVector(initialX, initialY, initialZ, initialA);
			case 5:
				return new CartesianVector(initialX, initialY, initialZ, initialA, initialB);
			case 6:
				return new CartesianVector(initialX, initialY, initialZ, initialA, initialB, initialC);
			default:
				return new CartesianVector(initialX, initialY, initialZ, initialA, initialB, initialC);
		}
	}

	/**
	 * Copies the value of an existing vector into the parent vector.
	 * 
	 * @param vector The vector to be copied.
	 */
	public void copy(CartesianVector vector)
	{
		x = vector.x;
		y = vector.y;
		for (int dimension = 3; dimension <= dimensions; dimension++)
		{
			switch (dimensions)
			{
				case 3:
					z = vector.z;
					break;
				case 4:
					a = vector.a;
					break;
				case 5:
					b = vector.b;
					break;
				case 6:
					c = vector.c;
					break;
			}
		}
	}

	/**
	 * Sets an existing vector to the specified values.
	 * 
	 * @param newX A double representing the new x value.
	 * @param newY A double representing the new y value.
	 */
	public void set(double newX, double newY)
	{
		x = newX;
		y = newY;
	}

	/**
	 * Sets an existing vector to the specified values.
	 * 
	 * @param newX A double representing the new x value.
	 * @param newY A double representing the new y value.
	 * @param newZ A double representing the new z value.
	 */
	public void set(double newX, double newY, double newZ)
	{
		x = newX;
		y = newY;
		z = newZ;
	}

	/**
	 * Sets an existing vector to the specified values.
	 * 
	 * @param newX A double representing the new x value.
	 * @param newY A double representing the new y value.
	 * @param newZ A double representing the new z value.
	 * @param newA A double representing the new a value.
	 */
	public void set(double newX, double newY, double newZ, double newA)
	{
		x = newX;
		y = newY;
		z = newZ;
		a = newA;
	}

	/**
	 * Sets an existing vector to the specified values.
	 * 
	 * @param newX A double representing the new x value.
	 * @param newY A double representing the new y value.
	 * @param newZ A double representing the new z value.
	 * @param newA A double representing the new a value.
	 * @param newB A double representing the new b value.
	 */
	public void set(double newX, double newY, double newZ, double newA, double newB)
	{
		x = newX;
		y = newY;
		z = newZ;
		a = newA;
		b = newB;
	}

	/**
	 * Sets an existing vector to the specified values.
	 * 
	 * @param newX A double representing the new x value.
	 * @param newY A double representing the new y value.
	 * @param newZ A double representing the new z value.
	 * @param newA A double representing the new a value.
	 * @param newB A double representing the new b value.
	 * @param newC A double representing the new c value.
	 */
	public void set(double newX, double newY, double newZ, double newA, double newB, double newC)
	{
		x = newX;
		y = newY;
		z = newZ;
		a = newA;
		b = newB;
		c = newC;
	}

	/**
	 * Creates a clone of the vector
	 * 
	 * @return An exact copy of the initial vector.
	 */
	public CartesianVector clone()
	{
		return vectorType(dimensions, x, y, z, a, b, c);
	}

	/**
	 * Gets the result of adding another vector to the parent vector. This function
	 * does NOT modify the parent vector.
	 * 
	 * @param vector The vector to add to the parent.
	 * @return A new vector containing the result of the addition. The result will
	 *         have the same dimensions as the parent.
	 */
	public CartesianVector getAddition(CartesianVector vector)
	{
		return vectorType(dimensions, x + vector.x, y + vector.y, z + vector.z, a + vector.a, b + vector.b, c + vector.c);
	}

	/**
	 * Gets the result of subtracting another vector from the parent vector. This
	 * function does NOT modify the parent vector.
	 * 
	 * @param vector The vector to subtract from the parent.
	 * @return A new vector containing the result of the subtraction. The result will
	 *         have the same dimensions as the parent.
	 */
	public CartesianVector getSubtraction(CartesianVector vector)
	{
		return vectorType(dimensions, x - vector.x, y - vector.y, z - vector.z, a - vector.a, b - vector.b, c - vector.c);
	}

	/**
	 * Gets the result of multiplying the parent vector by a number. This function
	 * does NOT modify the parent vector.
	 * 
	 * @param scalar A double that will be used to multiply each component of the
	 *               parent vector
	 * @return A new vector containing the result of the multiplication. The result will
	 *         have the same dimensions as the parent.
	 */
	public CartesianVector getMultiplication(double scalar)
	{
		return vectorType(dimensions, x * scalar, y * scalar, z * scalar, a * scalar, b * scalar, c * scalar);
	}

	/**
	 * Gets the result of dividing the parent vector by a number. This function does
	 * NOT modify the parent vector.
	 * 
	 * @param scalar A double that will be used to divide each component of the
	 *               parent vector.
	 * @return A new vector containing the result of the division. The result will
	 *         have the same dimensions as the parent.
	 */
	public CartesianVector getDivision(double scalar)
	{
		return vectorType(dimensions, x / scalar, y / scalar, z / scalar, a / scalar, b / scalar, c / scalar);
	}

	/**
	 * Gets the result of normalizing the parent vector to a length of 1. This
	 * function does NOT modify the parent vector. Calling this function also
	 * updates the parent vector's magnitude.
	 * 
	 * @return A new vector3D with the same direction as the parent, but with a
	 *         length of 1. The result will be the same type as the parent (2D or
	 *         3D).
	 */
	public CartesianVector getNormalization()
	{
		return getDivision(magnitude3D());
	}

	/**
	 * Adds another vector to the parent vector. This function DOES modify the
	 * parent vector.
	 * 
	 * @param vector The vector to add to the parent.
	 */
	public void add(CartesianVector vector)
	{
		x += vector.x;
		y += vector.y;
		for (int dimension = 3; dimension < dimensions; dimension++)
		{
			switch (dimensions)
			{
				case 3:
					z += vector.z;
					break;
				case 4:
					a += vector.a;
					break;
				case 5:
					b += vector.b;
					break;
				case 6:
					c += vector.c;
					break;
			}
		}
	}

	/**
	 * Subtracts another vector to the parent vector. This function DOES modify the
	 * parent vector.
	 * 
	 * @param vector The vector to subtract from the parent.
	 */
	public void subtract(CartesianVector vector)
	{
		x -= vector.x;
		y -= vector.y;
		for (int dimension = 3; dimension < dimensions; dimension++)
		{
			switch (dimensions)
			{
				case 3:
					z -= vector.z;
					break;
				case 4:
					a -= vector.a;
					break;
				case 5:
					b -= vector.b;
					break;
				case 6:
					c -= vector.c;
					break;
			}
		}
	}

	/**
	 * Multiplies the parent vector by a number. This function DOES modify the
	 * parent vector.
	 * 
	 * @param scalar A double that will be used to multiply each component of the
	 *               parent vector.
	 */
	public void multiply(double scalar)
	{
		x *= scalar;
		y *= scalar;
		for (int dimension = 3; dimension < dimensions; dimension++)
		{
			switch (dimensions)
			{
				case 3:
					z *= scalar;
					break;
				case 4:
					a *= scalar;
					break;
				case 5:
					b *= scalar;
					break;
				case 6:
					c *= scalar;
					break;
			}
		}
	}

	/**
	 * Divides the parent vector by a number. This function DOES modify the parent
	 * vector.
	 * 
	 * @param scalar A double that will be used to divide each component of the
	 *               parent vector.
	 */
	public void divide(double scalar)
	{
		x /= scalar;
		y /= scalar;
		for (int dimension = 3; dimension < dimensions; dimension++)
		{
			switch (dimensions)
			{
				case 3:
					z /= scalar;
					break;
				case 4:
					a /= scalar;
					break;
				case 5:
					b /= scalar;
					break;
				case 6:
					c /= scalar;
					break;
			}
		}
	}

	/**
	 * Normalizes the parent vector to a length of 1. This function DOES modify the
	 * parent vector. Calling this function also updates the parent vector's
	 * magnitude.
	 */
	public void normalize()
	{
		divide(magnitude3D());
	}

	/**
	 * Calculates magnitude of the parent vector.
	 * 
	 * @return A double representing the magnitude of the parent vector. This value
	 *         is also stored in the parent to avoid extra calculations when
	 *         possible.
	 */
	public double magnitude2D()
	{
		magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
		return magnitude;
	}

	/**
	 * Calculates magnitude of the parent vector.
	 * 
	 * @return A double representing the magnitude of the parent vector. This value
	 *         is also stored in the parent to avoid extra calculations when
	 *         possible.
	 */
	public double magnitude3D()
	{
		magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
		return magnitude;
	}

	/**
	 * Calculates the direction of the parent vector.
	 * 
	 * @return A double representing the direction of the parent vector in radians.
	 *         This value is also stored in the parent to avoid extra calculations
	 *         when possible.
	 */
	public double direction2D()
	{
		direction = Math.atan2(y, x);
		return direction;
	}

	/**
	 * Calculates average of the parent vector's components.
	 * 
	 * @return A double representing the average of the parent vector's components.
	 *         This is only useful in a few cases, so ensure this is the right
	 *         function. This value is also stored in the parent to avoid extra
	 *         calculations when possible.
	 */
	public double average()
	{
		average = x + y + z + a + b + c;
		average /= dimensions;
		return average;
	}
}
