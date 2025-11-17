#ifndef KLEIBER_DYNAMICS_QUADCOPTER_PARAMETERS_H_
#define KLEIBER_DYNAMICS_QUADCOPTER_PARAMETERS_H_

namespace quadcopter
{

struct VehicleParameters
{
    // Mass (kg)
    double m = 1.0;

    // Force and moment constants
    double K_f; // Force
    double K_m; // Moment

    /**
     * @brief Based on nothing but the feeling that torque of the props is much
     * smaller than the force of the props, we divide K_f by 20 to get K_m
     */
    double Kf_divisor_ = 20.0; // Divide

    // Inertia (kg / m^2)
    double Ixx = 0.007;
    double Ixy = 0.0;
    double Ixz = 0.0;
    double Iyy = 0.006;
    double Iyz = 0.0;
    double Izz = 0.009;

    // Dimensions (m)
    double dx_arm = 0.097;
    double dy_arm = 0.15;

    // Limits.

    /**
     * @brief Force and moment constants. F = K_f w^2, M = K_b w^2. TODO:
     * This is arbitrary, but sort of based on the motor specs. Max force
     * is 1.522 kg for a 5.1x3.1x3 propeller. This assumes 16V when we
     * have 14.8V equipped. Let's take 1.522 * 14.8 / 16 = 1.40785 kg to be max
     * pull. (Force = 1.40785 * 9.81 = 13.811).
     */
    double max_motor_force = 13.811; // Max motor force (N)

    /**
     * @brief Max RPM (no-load) is 2750 KV * 14.8V = 40,700 RPM -> 4262 rad/s.
     * Assuming a load makes for 80% max speed -> 3409.6 rad/s
     */
    double max_omega = 3409.6; // Max angular velocity (rad / s)

    /**
     * @brief Defaults to limit to changes of 40% every 0.01 sec
     */
    double motor_slew_rate = 0.4 / 0.01; // Max motor slew rate (% / s)
};

struct EnvironmentParameters
{
    // Gravity (m/s^2)
    double g = 9.81;

    // Disturbances
    double dist_mean = 0.0;
    double dist_stddev = 0.0;
};

}; // namespace quadcopter

#endif