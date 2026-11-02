import { useEffect, useMemo, useState } from 'react'
import { createPortal, useFrame, useLoader } from '@react-three/fiber'
import * as THREE from 'three'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import URDFLoader, { type URDFRobot } from 'urdf-loader'
import { DetectorModel } from './DetectorModel'
import { detectorQuaternion, rosToThree } from './sceneMath'
import type { TrajectoryFrame, Vec3 } from './types'

const BASE = `${import.meta.env.BASE_URL}assets/robot`
const DETECTOR_COMPONENT = 'metal_detector_link'
const GRIPPER_COMPONENTS = [
  'gripper_r_rh_p12_rn_base',
  'gripper_r_rh_p12_rn_r1',
  'gripper_r_rh_p12_rn_r2',
  'gripper_r_rh_p12_rn_l1',
  'gripper_r_rh_p12_rn_l2',
] as const

function paintCollisionComponent(
  robot: URDFRobot,
  component: string,
  isColliding: boolean,
) {
  const isDetector = component === DETECTOR_COMPONENT
  const highlightCollision = isColliding && !isDetector
  robot.links[component]?.traverse((child) => {
    if (!(child instanceof THREE.Mesh)) return
    const currentMaterials = Array.isArray(child.material) ? child.material : [child.material]
    const currentSignature = currentMaterials.map((material) => material.uuid).join(':')
    if (child.userData.paiCollisionMaterialSource !== currentSignature) {
      const cloned = currentMaterials.map((material) => material.clone())
      child.material = Array.isArray(child.material) ? cloned : cloned[0]
      child.userData.paiCollisionMaterialSource = cloned
        .map((material) => material.uuid)
        .join(':')
    }
    const materials = Array.isArray(child.material) ? child.material : [child.material]
    materials.forEach((material) => {
      const colored = material as THREE.Material & {
        color?: THREE.Color
        emissive?: THREE.Color
        emissiveIntensity?: number
      }
      colored.color?.set(highlightCollision ? '#ef4444' : isDetector ? '#9ca3af' : '#475569')
      colored.emissive?.set(highlightCollision ? '#7f1d1d' : '#111827')
      if (colored.emissiveIntensity != null) {
        colored.emissiveIntensity = highlightCollision ? 0.9 : isDetector ? 0.04 : 0.08
      }
    })
  })
}

interface RobotModelProps {
  frame: TrajectoryFrame
  detectionRadii: Vec3
  showDetector: boolean
  showDetectionVolume: boolean
  showCollisions: boolean
  cumulativeCollisionComponents: string[]
}

export function RobotModel({
  frame,
  detectionRadii,
  showDetector,
  showDetectionVolume,
  showCollisions,
  cumulativeCollisionComponents,
}: RobotModelProps) {
  const [robot, setRobot] = useState<URDFRobot | null>(null)

  useEffect(() => {
    let active = true
    const manager = new THREE.LoadingManager()
    const loader = new URDFLoader(manager)
    loader.packages = {
      ffw_description: `${BASE}/ffw_description`,
      pai_detector: `${import.meta.env.BASE_URL}assets/detector`,
    }
    loader
      .loadAsync(`${BASE}/ffw_description/urdf/ffw_sg2_web.urdf`)
      .then((loaded) => {
        if (!active) return
        loaded.traverse((child) => {
          if (child instanceof THREE.Mesh) {
            child.castShadow = true
            child.receiveShadow = true
          }
        })
        loaded.joints.lift_joint.ignoreLimits = true
        setRobot(loaded)
      })
      .catch((error: unknown) => {
        console.error('FFW-SG2 URDF load failed', error)
      })
    return () => {
      active = false
    }
  }, [])

  useEffect(() => {
    if (!robot) return
    robot.setJointValues({
      lift_joint: frame.robot.lift_position_m,
      head_joint1: 0,
      head_joint2: 0,
      ...Object.fromEntries(frame.robot.left_arm_joint_rad.map((value, index) => [`arm_l_joint${index + 1}`, value])),
      ...Object.fromEntries(frame.robot.right_arm_joint_rad.map((value, index) => [`arm_r_joint${index + 1}`, value])),
      ...Object.fromEntries(
        ['l', 'r'].flatMap((side) =>
          [1, 2, 3, 4].map((index) => [`gripper_${side}_joint${index}`, frame.robot.gripper_joint_rad]),
        ),
      ),
    })
  }, [frame, robot])

  useEffect(() => {
    if (!robot) return
    const detector = robot.links.metal_detector_link
    detector.visible = showDetector
  }, [robot, showDetector])

  const basePosition = rosToThree(frame.robot.base_position_m)
  const controlPoint = robot?.links.metal_detector_control_point
  const renderedPosition = useMemo(() => new THREE.Vector3(), [])
  const expectedPosition = useMemo(() => new THREE.Vector3(), [])
  const renderedOrientation = useMemo(() => new THREE.Quaternion(), [])

  useFrame(() => {
    if (!controlPoint || !robot) return
    const colliding = new Set(showCollisions ? cumulativeCollisionComponents : [])
    ;[DETECTOR_COMPONENT, ...GRIPPER_COMPONENTS].forEach((component) => {
      paintCollisionComponent(robot, component, colliding.has(component))
    })
    if (!window.__PAI_CAPTURE__) return
    controlPoint.getWorldPosition(renderedPosition)
    expectedPosition.set(...rosToThree(frame.position_m))
    controlPoint.getWorldQuaternion(renderedOrientation)
    window.__PAI_CAPTURE__.renderedDetectorPositionErrorM =
      renderedPosition.distanceTo(expectedPosition)
    window.__PAI_CAPTURE__.renderedDetectorOrientationErrorRad =
      renderedOrientation.angleTo(detectorQuaternion(frame.orientation_wxyz))
    const componentColors: Record<string, string> = {}
    ;[DETECTOR_COMPONENT, ...GRIPPER_COMPONENTS].forEach((component) => {
      const link = robot?.links[component]
      if (!link) componentColors[component] = '<missing-link>'
      link?.traverse((child) => {
        if (componentColors[component] || !(child instanceof THREE.Mesh)) return
        const material = Array.isArray(child.material) ? child.material[0] : child.material
        if ('color' in material && material.color instanceof THREE.Color) {
          componentColors[component] = `#${material.color.getHexString()}`
        } else {
          componentColors[component] = `<${material.type}>`
        }
      })
    })
    window.__PAI_CAPTURE__.collisionComponentColors = componentColors
    window.__PAI_CAPTURE__.rightGripperLinkNames = Object.keys(robot?.links ?? {}).filter(
      (name) => name.includes('rh_p12'),
    )
  })

  return (
    <group position={basePosition} rotation={[0, frame.robot.base_yaw_rad, 0]}>
      <group rotation={[-Math.PI / 2, 0, 0]}>
        {robot && <primitive object={robot} />}
        {controlPoint &&
          createPortal(
            <DetectorModel
              radii={detectionRadii}
              showVolume={showDetectionVolume}
            />,
            controlPoint,
          )}
        {controlPoint &&
          createPortal(
            <mesh visible={showDetector} frustumCulled={false}>
              <sphereGeometry args={[0.0125, 20, 14]} />
              <meshStandardMaterial
                color="#9ca3af"
                emissive="#111827"
                emissiveIntensity={0.04}
              />
            </mesh>,
            controlPoint,
          )}
        <RidgebackBase wheelAngles={frame.robot.wheel_angle_rad} />
      </group>
    </group>
  )
}

function RidgebackBase({ wheelAngles }: { wheelAngles: [number, number, number, number] }) {
  const paths = useMemo(
    () => [
      `${BASE}/ridgeback_description/meshes/body.stl`,
      `${BASE}/ridgeback_description/meshes/top.stl`,
      `${BASE}/ridgeback_description/meshes/side-cover.stl`,
      `${BASE}/ridgeback_description/meshes/end-cover.stl`,
      `${BASE}/ridgeback_description/meshes/lights.stl`,
      `${BASE}/ridgeback_description/meshes/wheel.stl`,
      `${BASE}/ridgeback_description/meshes/rocker.stl`,
    ],
    [],
  )
  const [body, top, side, end, lights, wheel, rocker] = useLoader(STLLoader, paths)
  const wheelPositions: [number, number, number][] = [
    [0.319, 0.2755, 0.0759],
    [0.319, -0.2755, 0.0759],
    [-0.319, 0.2755, 0.0759],
    [-0.319, -0.2755, 0.0759],
  ]
  return (
    <group>
      <mesh geometry={body} position={[0, 0, 0.0759]} castShadow receiveShadow>
        <meshStandardMaterial color="#202630" roughness={0.65} metalness={0.42} />
      </mesh>
      <mesh geometry={top} position={[0, 0, 0.0759]} castShadow>
        <meshStandardMaterial color="#2d3642" roughness={0.58} metalness={0.38} />
      </mesh>
      <mesh geometry={side} position={[0, 0, 0.0759]} castShadow>
        <meshStandardMaterial color="#eab308" roughness={0.55} />
      </mesh>
      <mesh geometry={side} position={[0, 0, 0.0759]} rotation={[0, 0, Math.PI]} castShadow>
        <meshStandardMaterial color="#eab308" roughness={0.55} />
      </mesh>
      <mesh geometry={end} position={[0, 0, 0.0759]} castShadow>
        <meshStandardMaterial color="#151a22" roughness={0.72} />
      </mesh>
      <mesh geometry={end} position={[0, 0, 0.0759]} rotation={[0, 0, Math.PI]} castShadow>
        <meshStandardMaterial color="#151a22" roughness={0.72} />
      </mesh>
      <mesh geometry={lights} position={[0, 0, 0.0759]}>
        <meshStandardMaterial color="#f8fafc" emissive="#fde68a" emissiveIntensity={0.5} />
      </mesh>
      {[0.319, -0.319].map((x) => (
        <mesh
          key={x}
          geometry={rocker}
          position={[x, 0, 0.0759]}
          rotation={[Math.PI / 2, 0, Math.PI / 2]}
          castShadow
        >
          <meshStandardMaterial color="#111827" roughness={0.78} metalness={0.18} />
        </mesh>
      ))}
      {wheelPositions.map((position, index) => (
        <group key={position.join(':')} position={position} rotation={[0, wheelAngles[index], 0]}>
          <mesh geometry={wheel} rotation={[Math.PI / 2, 0, 0]} castShadow receiveShadow>
            <meshStandardMaterial color="#111318" roughness={0.92} />
          </mesh>
        </group>
      ))}
    </group>
  )
}
