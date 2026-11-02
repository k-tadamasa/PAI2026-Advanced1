import { Suspense, useEffect, useMemo, useRef } from 'react'
import { Canvas, useFrame, useThree } from '@react-three/fiber'
import { Environment, Grid, OrbitControls } from '@react-three/drei'
import * as THREE from 'three'
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib'
import { HumanSurface } from './HumanSurface'
import { RobotModel } from './RobotModel'
import { rosToThree } from './sceneMath'
import { TrajectoryPath } from './TrajectoryPath'
import type { CameraMode, LayerVisibility, SceneData } from './types'

interface ScreeningSceneProps {
  data: SceneData
  frameIndex: number
  layers: LayerVisibility
  cameraMode: CameraMode
}

export function ScreeningScene({ data, frameIndex, layers, cameraMode }: ScreeningSceneProps) {
  const frame = data.trajectory[frameIndex]
  const firstCollisionComponentFrame = useMemo(() => {
    const result = new Map<string, number>()
    data.trajectory.forEach((trajectoryFrame, index) => {
      trajectoryFrame.collision_components.forEach((component) => {
        if (!result.has(component)) result.set(component, index)
      })
    })
    return result
  }, [data])
  const cumulativeCollisionComponents = useMemo(
    () => [...firstCollisionComponentFrame]
      .filter(([, firstFrame]) => firstFrame <= frameIndex)
      .map(([component]) => component),
    [firstCollisionComponentFrame, frameIndex],
  )
  return (
    <Canvas
      shadows
      dpr={[1, 2]}
      gl={{ antialias: true, preserveDrawingBuffer: true, powerPreference: 'high-performance' }}
      camera={{ position: [-1.7, 1.35, 2.55], fov: 39, near: 0.02, far: 30 }}
    >
      <color attach="background" args={['#07101e']} />
      <fog attach="fog" args={['#07101e', 4.5, 10]} />
      <ambientLight intensity={0.75} />
      <directionalLight
        position={[-2, 5, 3]}
        intensity={2.4}
        color="#dbeafe"
        castShadow
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
      />
      <directionalLight position={[4, 2, -3]} intensity={1.1} color="#67e8f9" />
      <Suspense fallback={null}>
        <HumanSurface
          data={data}
          frameIndex={frameIndex}
          showInspected={layers.inspected}
          showCollisions={layers.collisions}
        />
        {layers.trajectory && (
          <TrajectoryPath data={data} frameIndex={frameIndex} showCollisions={layers.collisions} />
        )}
        {layers.robot && (
          <RobotModel
            frame={frame}
            detectionRadii={data.detector.ellipsoid_radii_m}
            showDetector={layers.detector}
            showDetectionVolume={layers.detectionVolume}
            showCollisions={layers.collisions}
            cumulativeCollisionComponents={cumulativeCollisionComponents}
          />
        )}
        <Environment preset="warehouse" environmentIntensity={0.24} />
      </Suspense>
      <Grid
        args={[12, 12]}
        position={[0, -0.002, 0]}
        cellSize={0.25}
        cellThickness={0.5}
        cellColor="#1e3a5f"
        sectionSize={1}
        sectionThickness={1}
        sectionColor="#2563eb"
        fadeDistance={8}
        fadeStrength={1.2}
        infiniteGrid
      />
      <CameraRig mode={cameraMode} detectorPosition={frame.position_m} />
    </Canvas>
  )
}
function CameraRig({ mode, detectorPosition }: { mode: CameraMode; detectorPosition: [number, number, number] }) {
  const { camera } = useThree()
  const controls = useRef<OrbitControlsImpl | null>(null)
  const target = useMemo(() => new THREE.Vector3(), [])
  const desired = useMemo(() => new THREE.Vector3(), [])

  useFrame((_, delta) => {
    const detector = new THREE.Vector3(...rosToThree(detectorPosition))
    if (mode === 'front') {
      desired.set(-2.45, 1.02, 0.02)
      target.set(0.82, 0.94, 0)
    } else if (mode === 'detector') {
      desired.copy(detector).add(new THREE.Vector3(-0.72, 0.38, 0.72))
      target.copy(detector)
    } else {
      desired.set(-1.65, 1.35, 2.55)
      target.set(0.72, 0.88, 0)
    }
    const alpha = 1 - Math.exp(-Math.min(delta, 0.05) * 5.5)
    camera.position.lerp(desired, alpha)
    if (controls.current) {
      controls.current.target.lerp(target, alpha)
      controls.current.update()
    }
  })

  return (
    <OrbitControls
      ref={controls}
      makeDefault
      enableDamping
      dampingFactor={0.08}
      minDistance={0.45}
      maxDistance={7}
      maxPolarAngle={Math.PI / 2 - 0.03}
    />
  )
}
