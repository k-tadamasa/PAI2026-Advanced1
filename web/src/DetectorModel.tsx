import * as THREE from 'three'
import type { Vec3 } from './types'

interface DetectorModelProps {
  radii: Vec3
  showVolume: boolean
}

/**
 * Detection volume in metal_detector_control_point coordinates.
 *
 * RobotModel portals this group into the inspection-head centre derived from
 * MD.stl by body_screening. Keeping the ellipsoid there (instead of placing it
 * independently at the planned path point) prevents the MD and volume from
 * separating when the robot moves.
 */
export function DetectorModel({ radii, showVolume }: DetectorModelProps) {
  return (
    <group visible={showVolume}>
      <mesh scale={[radii[0], radii[1], radii[2]]} frustumCulled={false}>
        <sphereGeometry args={[1, 32, 20]} />
        <meshBasicMaterial
          color="#3b82f6"
          transparent
          opacity={0.12}
          depthWrite={false}
          side={THREE.DoubleSide}
        />
      </mesh>
      <mesh scale={[radii[0], radii[1], radii[2]]} frustumCulled={false}>
        <sphereGeometry args={[1, 20, 12]} />
        <meshBasicMaterial
          color="#bfdbfe"
          wireframe
          transparent
          opacity={0.07}
          depthWrite={false}
        />
      </mesh>
    </group>
  )
}
