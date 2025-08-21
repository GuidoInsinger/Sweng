mod error;
mod controller;
mod dynamics;
mod config;
mod planner;

use dynamics::Quadrotor;
use controller::PIDController;
use error::SimulationError;
use config::Config;
// use planner:: HoverPlanner;
use nalgebra::Vector3;

fn main() -> Result<(), SimulationError>{
    let config_str = "config/quad.yaml";
    let config = Config::from_yaml(config_str).expect("Failed to load configuration.");

   
    let mut quad = Quadrotor::new(
        1.0 / config.simulation.simulation_frequency as f32,
        config.quadrotor.mass,
        config.quadrotor.gravity,
        config.quadrotor.drag_coefficient,
        config.quadrotor.inertia_matrix,
    )?;

    let _pos_gains = config.pid_controller.pos_gains;
    let _att_gains = config.pid_controller.att_gains;
    let mut controller = PIDController::new(
        [_pos_gains.kp, _pos_gains.kd, _pos_gains.ki],
        [_att_gains.kp, _att_gains.kd, _att_gains.ki],
        config.pid_controller.pos_max_int,
        config.pid_controller.att_max_int,
        config.quadrotor.mass,
        config.quadrotor.gravity,
    );
    let desired_position = Vector3::<f32>::new(0.0, 0.0, 1.0);
    let desired_velocity = Vector3::<f32>::new(0.0, 0.0, 0.0);
    let desired_yaw: f32 = 0.0;

    let (thrust, calculated_desired_orientation) = controller.compute_position_control(
            &desired_position,
            &desired_velocity,
            desired_yaw,
            &quad.position,
            &quad.velocity,
            quad.time_step,
        );
    
    let torque = controller.compute_attitude_control(
            &calculated_desired_orientation,
            &quad.orientation,
            &quad.angular_velocity,
            quad.time_step,
        );
    quad.update_dynamics_with_controls_rk4(thrust, &torque);
    
    println!("{:?}", quad.position);

    Ok(())
} 