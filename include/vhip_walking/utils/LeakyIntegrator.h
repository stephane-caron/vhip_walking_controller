/* Copyright 2018-2019 CNRS-UM LIRMM
 *
 * \author Stéphane Caron
 *
 * This file is part of vhip_walking_controller.
 *
 * vhip_walking_controller is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vhip_walking_controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with vhip_walking_controller. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#pragma once

/** Leaky integrator.
 *
 * The output satisfies the differential equation:
 *
 *     yd(t) = x(t) - leakRate * y(t)
 *
 * A leaky integrator is implemented exactly as an exponential moving average,
 * but it is homogeneous to the integral of the input signal (rather than the
 * signal itself). See <https://en.wikipedia.org/wiki/Leaky_integrator>.
 *
 */
template <typename T>
struct LeakyIntegratorBase
{
  /** Add constant input for a fixed duration.
   *
   * \param value Constant input.
   *
   * \param dt Fixed duration.
   *
   */
  void add(const T & value, double dt)
  {
    integral_ = (1. - rate_ * dt) * integral_ + dt * value;
    if (saturation_ > 0.)
    {
      saturate();
    }
  }

  /** Evaluate the output of the integrator.
   *
   */
  const T & eval() const
  {
    return integral_;
  }

  /** Get leak rate.
   *
   */
  double rate() const
  {
    return rate_;
  }

  /** Set the leak rate of the integrator.
   *
   * \param rate New leak rate.
   *
   */
  void rate(double rate)
  {
    rate_ = rate;
  }

  /** Set output saturation. Disable by providing a negative value.
   *
   * \param s Output will saturate between -s and +s.
   *
   */
  void saturation(double s)
  {
    saturation_ = s;
  }

protected:
  /** Saturate output.
   *
   */
  virtual void saturate() = 0;

protected:
  T integral_;
  double rate_ = 0.1;
  double saturation_ = -1.;
};

template <typename T>
struct LeakyIntegrator : LeakyIntegratorBase<T>
{
  /** Empty constructor.
   *
   */
  LeakyIntegrator()
  {
    this->integral_.setZero();
  }

  /** Saturate output.
   *
   */
  virtual void saturate() override
  {
    for (unsigned i = 0; i < 3; i++)
    {
      if (this->integral_(i) < -this->saturation_)
      {
        this->integral_(i) = -this->saturation_;
      }
      else if (this->integral_(i) > this->saturation_)
      {
        this->integral_(i) = this->saturation_;
      }
    }
  }

  /** Reset integral to zero.
   *
   */
  void setZero()
  {
    this->integral_.setZero();
  }
};

template <>
struct LeakyIntegrator<double> : LeakyIntegratorBase<double>
{
  /** Empty constructor.
   *
   */
  LeakyIntegrator()
  {
    this->integral_ = 0.;
  }

  /** Saturate output.
   *
   */
  virtual void saturate() override
  {
    if (this->integral_ < -this->saturation_)
    {
      this->integral_ = -this->saturation_;
    }
    else if (this->integral_ > this->saturation_)
    {
      this->integral_ = this->saturation_;
    }
  }

  /** Reset integral to zero.
   *
   */
  void setZero()
  {
    this->integral_ = 0.;
  }
};
