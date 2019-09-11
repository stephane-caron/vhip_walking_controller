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

#include <eigen-lssol/LSSOL_LS.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/CoPTask.h>

#include <vhip_walking/Pendulum.h>
#include <vhip_walking/Contact.h>
#include <vhip_walking/Sole.h>
#include <vhip_walking/defs.h>
#include <vhip_walking/utils/LeakyIntegrator.h>
#include <vhip_walking/utils/rotations.h>
#include <vhip_walking/utils/stats.h>

namespace vhip_walking
{
  /** Template model with which stabilizer feedback is computed.
   *
   */
  enum class TemplateModel
  {
    LinearInvertedPendulum,
    VariableHeightInvertedPendulum
  };

  /** Walking stabilization based on linear inverted pendulum tracking.
   *
   * Stabilization bridges the gap between the open-loop behavior of the
   * pendulum state reference (feedforward controls) and feedback read from
   * state estimation. In our case, feedback is done on the DCM of the VHIP:
   *
   *    TODO: update formulas here
   *
   * Which boils down into corresponding formulas for the CoP and CoM
   * acceleration targets.
   *
   */
  struct Stabilizer
  {
    /* Maximum angular velocities for foot damping control. */
    static constexpr double MAX_FDC_RX_VEL = 0.2; // [rad] / [s]
    static constexpr double MAX_FDC_RY_VEL = 0.2; // [rad] / [s]
    static constexpr double MAX_FDC_RZ_VEL = 0.2; // [rad] / [s]

    /* Maximum gains for HRP4LIRMM in standing static equilibrium. */
    static constexpr double MAX_COM_XY_ADMITTANCE = 20.;
    static constexpr double MAX_COM_Z_ADMITTANCE = 0.1;
    static constexpr double MAX_COP_ADMITTANCE = 0.1;
    static constexpr double MAX_DCM_I_GAIN = 8.;
    static constexpr double MAX_DCM_P_GAIN = 3.;
    static constexpr double MIN_DCM_P_GAIN = 1.;
    static constexpr double MAX_DFZ_ADMITTANCE = 5e-4;
    static constexpr double MAX_ZMP_GAIN = 20.;

    /* Avoid low-pressure targets too close to contact switches */
    static constexpr double MIN_DS_PRESSURE = 15.; // [N]

    /* Saturate integrator in case of windup */
    static constexpr double MAX_AVERAGE_DCM_ERROR = 0.05; // [m]
    static constexpr double MAX_ALTCC_COM_OFFSET = 0.10; // [m]
    static constexpr double MAX_ZMPCC_COM_OFFSET = 0.05; // [m]

    /** Initialize stabilizer.
     *
     * \param robot Robot model.
     *
     * \param ref CoM state reference placeholder.
     *
     * \param dt Controller timestep.
     *
     */
    Stabilizer(const mc_rbdyn::Robot & robot, const Pendulum & ref, double dt);

    /** Add GUI panel.
     *
     * \param gui GUI handle.
     *
     */
    void addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui);

    /** Log stabilizer entries.
     *
     * \param logger Logger.
     *
     */
    void addLogEntries(mc_rtc::Logger & logger);

    /** Add tasks to QP solver.
     *
     * \param solver QP solver to add tasks to.
     *
     */
    void addTasks(mc_solver::QPSolver & solver);

    /** Disable all feedback components.
     *
     */
    void disable();

    /** Compute ZMP of a wrench in the ZMP frame computed by updateZMPFrame().
     *
     * \param wrench Wrench at the origin of the world frame.
     *
     */
    Eigen::Vector3d computeZMP(const sva::ForceVecd & wrench) const;

    /** Read configuration from dictionary.
     *
     */
    void configure(const mc_rtc::Configuration &);

    /** Detect foot touchdown based on both force and distance.
     *
     * \param footTask Swing foot task.
     *
     * \param contact Target contact.
     *
     */
    bool detectTouchdown(const std::shared_ptr<mc_tasks::force::CoPTask> footTask, const Contact & contact);

    /** Apply stored configuration.
     *
     */
    void reconfigure();

    /** Remove tasks from QP solver.
     *
     * \param solver QP solver to remove tasks from.
     *
     */
    void removeTasks(mc_solver::QPSolver & solver);

    /** Reset CoM and foot CoP tasks.
     *
     * \param robots Robots where the task will be applied.
     *
     */
    void reset(const mc_rbdyn::Robots & robots);

    /** Update QP task targets.
     *
     * This function is called once the reference has been updated.
     *
     */
    void run();

    /** Configure foot task for contact seeking.
     *
     * \param footTask One of leftFootTask or rightFootTask.
     *
     * This function has no effect when the measured pressure is already higher
     * than the target. Otherwise, it will set a positive admittance along the
     * z-axis of the contact frame.
     *
     */
    void seekTouchdown(std::shared_ptr<mc_tasks::force::CoPTask> footTask);

    /** Configure foot task for contact at a given location.
     *
     * \param footTask One of leftFootTask or rightFootTask.
     * 
     * \param contact Target contact location.
     *
     */
    void setContact(std::shared_ptr<mc_tasks::force::CoPTask> footTask, const Contact & contact);

    /** Configure foot task for swinging.
     *
     * \param footTask One of leftFootTask or rightFootTask.
     *
     * Foot target is reset to the current frame pose.
     *
     */
    void setSwingFoot(std::shared_ptr<mc_tasks::force::CoPTask> footTask);

    /** Get contact state.
     *
     */
    ContactState contactState()
    {
      return contactState_;
    }

    /** Set desired contact state.
     *
     */
    void contactState(ContactState contactState)
    {
      contactState_ = contactState;
    }

    /** Update real-robot state.
     *
     * \param com Position of the center of mass.
     *
     * \param comd Velocity of the center of mass.
     *
     * \param wrench Net contact wrench in the inertial frame.
     *
     * \param leftFootRatio Desired pressure distribution ratio for left foot.
     *
     */
    void updateState(const Eigen::Vector3d & com, const Eigen::Vector3d & comd, const sva::ForceVecd & wrench, double leftFootRatio);

    /** Update H-representation of contact wrench cones.
     *
     * \param sole Sole parameters.
     *
     * See <https://hal.archives-ouvertes.fr/hal-02108449/document> for
     * technical details on the derivation of this formula.
     *
     */
    void wrenchFaceMatrix(const Sole & sole)
    {
      double X = sole.halfLength;
      double Y = sole.halfWidth;
      double mu = sole.friction;
      wrenchFaceMatrix_ <<
        // mx,  my,  mz,  fx,  fy,            fz,
            0,   0,   0,  -1,   0,           -mu,
            0,   0,   0,  +1,   0,           -mu,
            0,   0,   0,   0,  -1,           -mu,
            0,   0,   0,   0,  +1,           -mu,
           -1,   0,   0,   0,   0,            -Y,
           +1,   0,   0,   0,   0,            -Y,
            0,  -1,   0,   0,   0,            -X,
            0,  +1,   0,   0,   0,            -X,
          +mu, +mu,  -1,  -Y,  -X, -(X + Y) * mu,
          +mu, -mu,  -1,  -Y,  +X, -(X + Y) * mu,
          -mu, +mu,  -1,  +Y,  -X, -(X + Y) * mu,
          -mu, -mu,  -1,  +Y,  +X, -(X + Y) * mu,
          +mu, +mu,  +1,  +Y,  +X, -(X + Y) * mu,
          +mu, -mu,  +1,  +Y,  -X, -(X + Y) * mu,
          -mu, +mu,  +1,  -Y,  +X, -(X + Y) * mu,
          -mu, -mu,  +1,  -Y,  -X, -(X + Y) * mu;
    }

    /** ZMP target after force distribution.
     *
     */
    Eigen::Vector3d zmp() const
    {
      return computeZMP(distribWrench_);
    }

    /** Vertices of the ZMP support polygon in the world frame.
     *
     */
    const std::vector<Eigen::Vector3d> & zmpPolygon()
    {
      return zmpPolygon_;
    }

  private:
    /** Weights for force distribution quadratic program (FDQP).
     *
     */
    struct FDQPWeights
    {
      /** Read force distribution QP weights from configuration.
       *
       * \param config Configuration dictionary.
       *
       */
      void configure(const mc_rtc::Configuration & config)
      {
        double ankleTorqueWeight = config("ankle_torque");
        double netWrenchWeight = config("net_wrench");
        double pressureWeight = config("pressure");
        ankleTorqueSqrt = std::sqrt(ankleTorqueWeight);
        netWrenchSqrt = std::sqrt(netWrenchWeight);
        pressureSqrt = std::sqrt(pressureWeight);
      }

    public:
      double ankleTorqueSqrt;
      double netWrenchSqrt;
      double pressureSqrt;
    };

    /** Check that all gains are within boundaries.
     *
     */
    void checkGains();

    /** Check whether the robot is in the air.
     *
     */
    void checkInTheAir();

    /** Compute desired wrench using the LIP template model.
     *
     */
    sva::ForceVecd computeLIPDesiredWrench();

    /** Compute desired wrench using the VHIP template model.
     *
     */
    sva::ForceVecd computeVHIPDesiredWrench();

    /** Distribute a desired wrench to supporting contacts.
     *
     * \param desiredWrench Desired resultant reaction wrench.
     *
     */
    void distributeWrench(const sva::ForceVecd & desiredWrench);

    /** Distribute a desired wrench in double support.
     *
     * \param desiredWrench Desired resultant reaction wrench.
     *
     */
    void distributeWrenchDS(const sva::ForceVecd & desiredWrench);

    /** Project desired wrench to the support foot in SSP.
     *
     * \param desiredWrench Desired resultant reaction wrench.
     *
     * \param footTask Target foot.
     *
     */
    void distributeWrenchSS(const sva::ForceVecd & desiredWrench, std::shared_ptr<mc_tasks::force::CoPTask> & footTask);

    /** Reset admittance, damping and stiffness for every foot in contact.
     *
     * \note This function is called at every run().
     *
     */
    void updateSupportFootGains();

    /** Update CoM task based on horizontal and vertical admittance control.
     *
     * Center of mass motions are added to the reference to compensate errors
     * in the three components of the net contact wrench that the VHIP
     * represents (ZMP \f$r\f$ and normalized stiffness \f$\lambda\f$).
     * Horizontal translations improve tracking of the ZMP, while vertical
     * translations improve tracking of the normalized stiffness.
     *
     */
    void updateCoMAdmittanceControl();

    /** Update CoM horizontal admittance control.
     *
     * ZMP compensation by horizontal CoM motions is based on
     * "体幹位置コンプライアンス制御によるモデル誤差吸収", Section 6.2.2 of Dr
     * Nagasaka's PhD thesis (1999) available from:
     * <https://sites.google.com/site/humanoidchannel/home/publication>. We
     * follow the same approach but use CoM damping control with an internal
     * leaky integrator.
     *
     * \note ZMPCC stands for "ZMP Compensation Control".
     *
     */
    void updateCoMZMPCC();

    /** Update CoM vertical admittance control.
     *
     * This is an original idea we experiment in this controller. The altitude
     * of the CoM (i.e. its coordinate in [m] in the direction of gravity) is
     * regulated to track the VHIP normalized stiffness \f$\lambda\f$ (relating
     * CoM-ZMP deviations to CoM acceleration by \f$\ddot{c} = \lambda (c -
     * r)\f$).
     *
     */
    void updateCoMAltitude();

    /** Apply foot force difference control.
     *
     * This method is described in Section III.E of "Biped walking
     * stabilization based on linear inverted pendulum tracking" (Kajita et
     * al., IROS 2010).
     *
     */
    void updateFootForceDifferenceControl();

    /** Update ZMP frame from contact state.
     *
     */
    void updateZMPFrame();

    /** Get 6D contact admittance vector from 2D CoP admittance.
     *
     */
    sva::ForceVecd contactAdmittance()
    {
      return {{copAdmittance_.y(), copAdmittance_.x(), 0.}, {0., 0., 0.}};
    }

  public:
    Contact leftFootContact;
    Contact rightFootContact;
    std::shared_ptr<mc_tasks::CoMTask> comTask;
    std::shared_ptr<mc_tasks::force::CoPTask> leftFootTask;
    std::shared_ptr<mc_tasks::force::CoPTask> rightFootTask;

  private:
    ContactState contactState_ = ContactState::DoubleSupport;
    Eigen::HrepXd zmpArea_;
    Eigen::LSSOL_LS leastSquares_; /**< Least-squares solver for wrench distribution */
    Eigen::Matrix<double, 16, 6> wrenchFaceMatrix_;
    Eigen::Vector2d copAdmittance_ = Eigen::Vector2d::Zero();
    Eigen::Vector3d comAdmittance_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d comOffset_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d comStiffness_ = {1000., 1000., 100.}; /**< Stiffness of CoM IK task */
    Eigen::Vector3d dcmAverageError_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d dcmError_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d measuredCoM_;
    Eigen::Vector3d measuredCoMd_;
    Eigen::Vector3d measuredZMP_;
    Eigen::Vector3d vhipDCM_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d vhipZMP_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d zmpccCoMAccel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d zmpccCoMOffset_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d zmpccCoMVel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d zmpccError_ = Eigen::Vector3d::Zero();
    ExponentialMovingAverage dcmIntegrator_;
    FDQPWeights fdqpWeights_;
    LeakyIntegrator<Eigen::Vector3d> zmpccIntegrator_;
    LeakyIntegrator<double> altccIntegrator_;
    TemplateModel model_ = TemplateModel::VariableHeightInvertedPendulum;
    bool inTheAir_ = false; /**< Is the robot in the air? */
    bool zmpccOnlyDS_ = true; /**< Apply ZMPCC only during double support phases? */
    const Pendulum & pendulum_; /**< Reference to desired template model state */
    const mc_rbdyn::Robot & controlRobot_; /**< Control robot model (input to joint position controllers) */
    double altccCoMAccel_ = 0.;
    double altccCoMOffset_ = 0.;
    double altccCoMVel_ = 0.;
    double altccError_ = 0.;
    double comWeight_ = 1000.; /**< Weight of CoM IK task */
    double contactWeight_ = 100000.; /**< Weight of contact IK tasks */
    double dcmGain_ = 1.25; /**< Proportional gain \f$k > 1\f$ such that \f$\Delta r = k \Delta \xi \Leftrightarrow \Delta\xi = \omega (1 - k) \Delta \xi\f$ */
    double dcmIntegralGain_ = 2.5; /**< Integral gain on DCM error */
    double dfzAdmittance_ = 1e-4; /**< Admittance for vertical foot force control */
    double distribLambda_ = 0.; /**< Normalized stiffness required by FDQP */
    double dt_ = 0.005; /**< Controller cycle in [s] */
    double fdqpRunTime_ = 0.; /**< Measured average duration in [s] of a call to distributeWrench() */
    double leftFootRatio_ = 0.5; /**< Desired ratio of total normal foot force applied to left foot */
    double logMeasuredDFz_ = 0.; /**< Measured vertical force difference between left and right foot */
    double logMeasuredSTz_ = 0.; /**< Model vertical position average between left and right foot */
    double logTargetDFz_ = 0.; /**< Desired vertical force difference between left and right foot */
    double logTargetSTz_ = 0.; /**< Desired vertical position average between left and right foot */
    double mass_ = 38.; /**< Robot mass in [kg] */
    double measuredLambda_ = 0.; /**< Normalized stiffness measured from sensors */
    double runTime_ = 0.; /**< Measured average duration in [s] of a call to run() */
    double swingFootStiffness_ = 2000.; /**< Stiffness of swing foot IK task */
    double swingFootWeight_ = 500.; /**< Weight of swing foot IK task */
    double vdcDamping_ = 0.; /**< Vertical Drift Compensation damping */
    double vdcFrequency_ = 1.; /**< Vertical Drift Compensation frequency */
    double vdcStiffness_ = 1000.; /**< Vertical Drift Compensation stiffness */
    double vdcZPos_ = 0.;
    double vfcZCtrl_ = 0.;
    double vhipLambda_ = 0.;
    double vhipOmega_ = 0.;
    double vhipRunTime_ = 0.; /**< Measured average duration in [s] of a call to computeVHIPDesiredWrench() */
    mc_rtc::Configuration config_; /**< Stabilizer configuration dictionary */
    std::vector<Eigen::Vector3d> zmpPolygon_; /**< Vertices of the ZMP support polygon in the world frame */
    std::vector<std::string> comActiveJoints_; /**< Joints used by CoM IK task */
    sva::ForceVecd distribWrench_ = sva::ForceVecd::Zero();
    sva::ForceVecd measuredWrench_; /**< Measured net contact wrench in the world frame */
    sva::MotionVecd contactDamping_;
    sva::MotionVecd contactStiffness_;
    sva::PTransformd zmpFrame_;
  };
}
