export type Vec3 = [number, number, number]
export type Vec4 = [number, number, number, number]

export interface RobotFrame {
  time_s: number
  base_position_m: Vec3
  base_yaw_rad: number
  lift_position_m: number
  left_arm_joint_rad: [number, number, number, number, number, number, number]
  right_arm_joint_rad: [number, number, number, number, number, number, number]
  gripper_joint_rad: number
  wheel_angle_rad: [number, number, number, number]
  inspection_normal_yaw_rad: number
  inspection_normal_pitch_rad: number
}

export interface TrajectoryFrame {
  index: number
  position_m: Vec3
  surface_normal: Vec3
  orientation_wxyz: Vec4
  collision: boolean
  colliding_face_ids: number[]
  collision_components: string[]
  newly_inspected_face_ids: number[]
  coverage_ratio: number
  robot: RobotFrame
}

export interface SceneData {
  schema_version: string
  title: string
  account: { name: string; id: string }
  method: string
  scope: {
    front_only: boolean
    human3d_part_ids: number[]
    collision_affects_planning: boolean
    robot_model: string
    robot_motion: string
    collision_model: string
  }
  parameters: {
    standoff_m: number
    ellipsoid_radii_m: Vec3
    target_coverage_ratio: number
    lane_count: number
    lane_spacing_scale: number
    central_lane_gap_scale: number
    flip_md_at_center: boolean
    lane_half_width_m: number
    local_fit_half_window: number
    interpolation_gap_m: number
    loess_window_points: number
    lane_reference_y_m: number[]
    front_normal_max_angle_deg: number
    front_width_fraction: number
    front_top_margin_m: number
    maximum_orientation_step_deg: number
    sample_period_s: number
    source_sample_period_s?: number
    scan_speed_scale?: number
    maximum_control_position_step_m?: number
    maximum_control_orientation_step_deg?: number
    maximum_control_base_step_m?: number
    maximum_control_base_yaw_step_deg?: number
    maximum_control_joint_step_deg?: number
    maximum_control_lift_step_m?: number
    control_time_step_median_s?: number
    control_pose_rate_average_hz?: number
  }
  metrics: {
    coverage_ratio: number
    target_area_m2: number
    inspected_area_m2: number
    path_length_m: number
    planning_time_s: number
    control_point_count: number
    trajectory_pose_count: number
    collision_pose_count: number
    source_trajectory_pose_count?: number
    maximum_subdivisions_per_source_interval?: number
    inspection_time_s?: number
    detector_collision_pose_count: number
    gripper_collision_pose_count: number
    stop_reason: string
  }
  human_mesh: {
    vertices: Vec3[]
    faces: [number, number, number][]
    face_part_ids: number[]
    target_face_ids: number[]
  }
  detector: {
    visual_asset: string
    collision_asset: string
    ellipsoid_radii_m: Vec3
    standoff_m: number
  }
  robot: {
    id: string
    motion_model: string
    wheel_radius_m: number
    wheel_positions_m: [number, number]
    arm_visualization: string
    collision_components: string[]
    scan_speed_scale?: number
    control_pose_count?: number
  }
  collision: {
    model: string
    method: string
    contact_tolerance_m: number
    affects_planning: boolean
  }
  trajectory: TrajectoryFrame[]
}

export interface LayerVisibility {
  robot: boolean
  detector: boolean
  detectionVolume: boolean
  inspected: boolean
  trajectory: boolean
  collisions: boolean
  labels: boolean
}

export type CameraMode = 'overview' | 'front' | 'detector'

declare global {
  interface Window {
    __PAI_CAPTURE__?: {
      ready: boolean
      frameCount: number
      setFrame: (frame: number) => void
      setCameraMode: (mode: CameraMode) => void
      playAtSpeed: (speed: number) => void
      renderedDetectorPositionErrorM?: number
      renderedDetectorOrientationErrorRad?: number
      collisionComponentColors?: Record<string, string>
      rightGripperLinkNames?: string[]
      cumulativeCollisionFaceCount?: number
    }
    __PAI_PRESENTATION__?: {
      ready: boolean
      duration: number
      done: boolean
      start: () => void
    }
  }
}
