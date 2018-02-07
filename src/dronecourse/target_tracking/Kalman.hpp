/**
 * @file TargetTracker.hpp
 * Linear Discrete Kalman filter
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once

#include <matrix/math.hpp>
#include <stdint.h>

// M = number of states
// N = number of measurements
template<uint8_t M, uint8_t N>
class KalmanFilter
{

public:
	KalmanFilter() {};

	/**
	 * Initialize Kalman filter
	 *
	 * @param f   dynamic matrix F
	 * @param w   standard deviation of system noise
	 * @param h   design (measurement) matrix H
	 * @param v   standard deviation of measurement noise v
	 * @param x0  initial state x(k=0)
	 * @param p0  standard deviation of initial state
	 * @param dt  update period Delta t
	 */
	void init(const matrix::SquareMatrix<float, M> &f,
		  const matrix::Vector<float, M> &w,
		  const matrix::Matrix<float, N, M> &h,
		  const matrix::Vector<float, M> &x0,
		  const matrix::Vector<float, M> &p0,
		  float dt)
	{
		// --------------------------------------------------------------------------
		// TODO initialize member functions _h, _h_t, _dt, _f and _x
		// --------------------------------------------------------------------------

		// --------------------------------------------------------------------------
		// TODO set covariance matrix of estimation P(k=0)
		// --------------------------------------------------------------------------

		// Set system noise
		setSystemNoise(w);
	};

	/**
	 * Perform a prediction step of the filter
	 */
	void predict()
	{
		// ------------------------------------------
		// TODO perform prediction: update _x and _p
		// ------------------------------------------
	};

	/**
	 * Perform a correction (update) step of the filter
	 *
	 * @param z    measurement
	 * @param v    covariance matrix of measurement
	 */
	void correct(const matrix::Vector<float, N> &z, const matrix::SquareMatrix<float, N> &r)
	{
		// -------------------------------------------------
		// TODO calc kalman gain k (weight/trust of measurement)
		// -------------------------------------------------

		// --------------------------------------------------
		// TODO update state estimation
		// --------------------------------------------------

		// --------------------------------------------------
		// TODO update estimation of state covariance
		// --------------------------------------------------
	}


	/**
	 * Return estimated state
	 */
	const matrix::Vector<float, M> &getStateEstimate() const
	{
		// ------------------------------------
		// TODO return state estimation
		// ------------------------------------
		return NULL; // replace this line
	};

	/**
	 * Return variance (std^2) of state estimation
	 */
	matrix::Vector<float, M> getStateVariances() const
	{
		// -----------------------------------------
		// TODO return variance of state estimation
		//      only diagonal elements
		// -----------------------------------------
		return matrix::Vector<float, M>(); // replace this line
	}

	/**
	 * Set system noise
	 * This calculates the state transition matrix _phi
	 * and the covariance matrix of the system noise _q
	 *
	 * @param w   standard deviation of system noise w
	 */
	void setSystemNoise(const matrix::Vector<float, M> &w)
	{
		// ------------------------------------------------
		// TODO calculate state transition matrix _phi
		// and the covariance matrix of the system noise q
		// Hint: use matrix::expm(A) to compute matrix exponential of A
		// ------------------------------------------------
	}

private:
	/** Covariance of system (model) noise [constant] */
	matrix::SquareMatrix<float, M> _q;
	/** measurement (design) matrix [constant] */
	matrix::Matrix<float, N, M> _h;
	/** state transition matrix [constant] */
	matrix::SquareMatrix<float, M> _phi;

	/** Dynamic matrix (only stored to recalculated phi and q in setSystemNoise(..)) */
	matrix::SquareMatrix<float, M> _f;
	/** time increment (only stored to recalculated phi and q in setSystemNoise(..)) */
	float _dt;

	/** A posteriori (estimated) state covariance */
	matrix::SquareMatrix<float, M> _p;
	/** A posteriori (estimated) state */
	matrix::Vector<float, M> _x;

	// transposed matrices for faster calculation
	/** Transposed state transition matrix [constant] */
	matrix::SquareMatrix<float, M> _phi_t;
	/** Transposed measurement (design) matrix [constant] */
	matrix::Matrix<float, M, N> _h_t;
};
