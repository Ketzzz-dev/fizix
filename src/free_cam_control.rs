use three_d::{
    Camera, CameraAction, CameraControl, Event, Key, Matrix3, SquareMatrix, Vector3, Zero,
};

pub struct FreeCamControl {
    control: CameraControl,

    speed: f32,
    acceleration: f32,
    deceleration: f32,

    direction: Vector3<f32>,
    velocity: Vector3<f32>,
}

impl FreeCamControl {
    pub fn new(speed: f32, acceleration: f32, deceleration: f32) -> Self {
        Self {
            control: CameraControl {
                right_drag_horizontal: CameraAction::Yaw {
                    speed: -std::f32::consts::PI / 1800.0,
                },
                right_drag_vertical: CameraAction::Pitch {
                    speed: -std::f32::consts::PI / 1800.0,
                },

                ..Default::default()
            },

            speed,
            acceleration,
            deceleration,

            direction: Vector3::zero(),
            velocity: Vector3::zero()
        }
    }

    pub fn handle_events(&mut self, camera: &mut Camera, events: &mut [Event]) -> bool {
        for event in events.iter() {
            match *event {
                Event::KeyPress { kind, .. } => self.handle_keys(kind, true),
                Event::KeyRelease { kind, .. } => self.handle_keys(kind, false),

                _ => {}
            }
        }

        self.control.handle_events(camera, events)
    }
    pub fn update(&mut self, camera: &mut Camera, delta_time: f64) {
        let camera_transform = camera.view();
        let rotation_matrix = Matrix3::new(
            camera_transform.x.x, camera_transform.x.y, camera_transform.x.z,
            camera_transform.y.x, camera_transform.y.y, camera_transform.y.z,
            camera_transform.z.x, camera_transform.z.y, camera_transform.z.z
        ).invert().unwrap();

        let target_velocity = rotation_matrix * self.direction * self.speed;
        let delta_velocity = target_velocity - self.velocity;
        let acceleration_rate = if self.direction.is_zero() {
            self.deceleration
        } else {
            self.acceleration
        };

        self.velocity += delta_velocity * acceleration_rate * delta_time as f32;

        camera.translate(&(self.velocity * delta_time as f32));
    }

    fn handle_keys(&mut self, key: Key, pressed: bool) {
        match key {
            Key::W => self.direction.z = if pressed { -self.speed } else { 0.0 },
            Key::S => self.direction.z = if pressed { self.speed } else { 0.0 },
            Key::A => self.direction.x = if pressed { -self.speed } else { 0.0 },
            Key::D => self.direction.x = if pressed { self.speed } else { 0.0 },
            Key::Q => self.direction.y = if pressed { -self.speed } else { 0.0 },
            Key::E => self.direction.y = if pressed { self.speed } else { 0.0 },

            _ => {}
        }
    }
}
