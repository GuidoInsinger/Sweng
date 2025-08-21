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
use rerun;
fn main() -> Result<(), SimulationError>{
    let config_str = "config/quad.yaml";
    let config = Config::from_yaml(config_str).expect("Failed to load configuration.");

    let time_step = 1.0 / config.simulation.simulation_frequency as f32;

    let mut quads: Vec<Quadrotor> = (0..config.simulation.n_drones)
        .map(|_| Quadrotor::new(
            time_step,
            config.quadrotor.mass,
            config.quadrotor.gravity,
            config.quadrotor.drag_coefficient,
            config.quadrotor.inertia_matrix,
        ))
        .collect::<Result<_, _>>()?; 

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

    let rec = rerun::RecordingStreamBuilder::new("swarmeng").spawn()?;
    rec.log_static("world", &rerun::ViewCoordinates::RIGHT_HAND_Z_UP())?;
    
    
    rec.set_duration_secs("time", 0.0);
    // rec.log(
    //     "world/box",
    //     &rerun::Boxes3D::from_half_sizes([[1.0, 1.0, 1.0]]),
    // )?;
    for i in 1..=config.simulation.n_drones {
        rec.log(format!("world/asset{i}").as_str(), &rerun::Asset3D::from_file_path("src/quadrotor_base.stl")?)?;
        // rec.log(format!("world/asset{i}").as_str(), &rerun::Points3D::new([(0.0, 0.0, 0.0)]))?;
    }

    let mut i = 0;
    loop {
        let time = time_step * i as f32;
        rec.set_duration_secs("time", time);
        
        for (j,quad) in quads.iter_mut().enumerate(){
            let desired_position = Vector3::<f32>::new(j as f32, j as f32, j as f32);
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
            quad.update_dynamics_with_controls_euler(thrust, &torque);
            let k = j+1;
            
            rec.log(
        format!("world/asset{k}").as_str(),
        &rerun::Transform3D::from_translation_rotation(
            rerun::Vec3D::new(quad.position.x, quad.position.y, quad.position.z),
            rerun::Quaternion::from_xyzw([
                quad.orientation.i,
                quad.orientation.j,
                quad.orientation.k,
                quad.orientation.w,
            ]),
        )
    )?;
            // println!("Drone {} position {:?}", j, quad.position);
        }
        i += 1;
            if time >= config.simulation.duration {
                println!("Finish");
                break;
            }
    }
    Ok(())
} 