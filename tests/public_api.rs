use manif_geom_rs::{na, SE2d, SE3d, SO2d, SO3d};

#[test]
fn crate_root_exports_are_usable_by_consumers() {
    let q2 = SO2d::from_angle(&0.25);
    let x2 = SE2d::from_parts(na::Vector2::new(1.0, 2.0), q2);
    let point2 = x2 * na::Vector2::new(3.0, 4.0);
    assert!(point2.iter().all(|value| value.is_finite()));

    let q3 = SO3d::from_euler(&0.1, &0.2, &0.3);
    let x3 = SE3d::from_parts(na::Vector3::new(1.0, 2.0, 3.0), q3);
    let point3 = x3 * na::Vector3::new(4.0, 5.0, 6.0);
    assert!(point3.iter().all(|value| value.is_finite()));
}

#[test]
fn public_modules_remain_available_for_explicit_imports() {
    let _: manif_geom_rs::so2::SO2<f32> = Default::default();
    let _: manif_geom_rs::se2::SE2<f32> = Default::default();
    let _: manif_geom_rs::so3::SO3<f32> = Default::default();
    let _: manif_geom_rs::se3::SE3<f32> = Default::default();
}
