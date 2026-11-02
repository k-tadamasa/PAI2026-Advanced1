import { useMemo, useRef } from 'react'
import { Canvas, useFrame, useThree } from '@react-three/fiber'
import { Grid } from '@react-three/drei'
import * as THREE from 'three'
import { rosToThree } from './sceneMath'
import type { Vec3 } from './types'

export interface PresentationData {
  schema_version: string
  subject: {
    id: string
    pose: string
    includes_rendered_garments: boolean
    vertices: Vec3[]
    faces: [number, number, number][]
    meshes: Array<{
      prim_path: string
      vertex_offset: number
      vertex_count: number
      triangle_offset: number
      triangle_count: number
    }>
  }
  acquisition: {
    backend: string
    point_count: number
    displayed_point_count: number
    points: Vec3[]
    colors: Vec3[]
  }
  human3d: { points: Vec3[]; part_ids: number[]; point_count: number }
  smplx: {
    vertices: Vec3[]
    faces: [number, number, number][]
    body_model: string
    fitting_loss: number
  }
  metrics: {
    coverage_ratio: number
    inspection_time_s: number
    path_length_m: number
    planning_time_s: number
    pose_estimation_time_s: number
    collision_pose_count: number
    scan_speed_scale: number
    trajectory_pose_count: number
  }
}

const PART_COLORS = [
  '#64748b', '#ea580c', '#e11d48', '#c026d3', '#7c3aed', '#4338ca',
  '#0284c7', '#0891b2', '#0f766e', '#059669', '#65a30d', '#ca8a04',
  '#d97706', '#dc2626', '#047857', '#0e7490',
]

export function PresentationScene({
  data,
  segment,
  elapsed,
  duration,
}: {
  data: PresentationData
  segment: 'subject' | 'pipeline'
  elapsed: number
  duration: number
}) {
  const rawOpacity = segment === 'pipeline' ? 1 - smoothstep(6.90, 7.10, elapsed) : 0
  const humanOpacity = segment === 'pipeline'
    ? smoothstep(6.90, 7.10, elapsed) * (1 - smoothstep(14.7, 15.3, elapsed))
    : 0
  const meshOpacity = segment === 'pipeline' ? smoothstep(14.7, 15.3, elapsed) : 0

  return (
    <Canvas
      dpr={[1, 2]}
      gl={{ antialias: true, preserveDrawingBuffer: true, powerPreference: 'high-performance' }}
      camera={{ position: [-1.3, 1.2, 2.7], fov: 35, near: 0.02, far: 30 }}
    >
      <color attach="background" args={['#07101e']} />
      <fog attach="fog" args={['#07101e', 5.5, 11]} />
      <ambientLight intensity={1.15} />
      <directionalLight position={[-2, 5, 3]} intensity={2.8} color="#e0f2fe" />
      <directionalLight position={[4, 2, -3]} intensity={1.5} color="#67e8f9" />
      {segment === 'subject' && <SubjectMesh data={data} />}
      {segment === 'pipeline' && (
        <>
          <PointCloud
            points={data.acquisition.points}
            colors={data.acquisition.colors}
            opacity={rawOpacity}
            size={0.010}
          />
          <SemanticPointCloud
            points={data.human3d.points}
            partIds={data.human3d.part_ids}
            opacity={humanOpacity}
          />
          <SmplxMesh data={data} opacity={meshOpacity} />
        </>
      )}
      <Grid
        args={[12, 12]}
        position={[0, -0.002, 0]}
        cellSize={0.25}
        cellThickness={0.45}
        cellColor="#1e3a5f"
        sectionSize={1}
        sectionThickness={1}
        sectionColor="#2563eb"
        fadeDistance={7}
        fadeStrength={1.3}
        infiniteGrid
      />
      <PresentationCamera segment={segment} elapsed={elapsed} duration={duration} />
    </Canvas>
  )
}

function SubjectMesh({ data }: { data: PresentationData }) {
  const geometry = useMemo(() => {
    const positions = new Float32Array(data.subject.faces.length * 9)
    const colors = new Float32Array(data.subject.faces.length * 9)
    const materialColor = (faceIndex: number) => {
      const group = data.subject.meshes.find(
        (mesh) => faceIndex >= mesh.triangle_offset && faceIndex < mesh.triangle_offset + mesh.triangle_count,
      )
      const name = group?.prim_path.toLowerCase() ?? ''
      if (name.includes('skin')) return new THREE.Color('#9f746b')
      if (name.includes('shirt')) return new THREE.Color('#315b82')
      if (name.includes('pants')) return new THREE.Color('#27364a')
      if (name.includes('shoe')) return new THREE.Color('#111827')
      return new THREE.Color('#94a3b8')
    }
    data.subject.faces.forEach((face, faceIndex) => {
      const color = materialColor(faceIndex)
      face.forEach((vertexIndex, corner) => {
        positions.set(rosToThree(data.subject.vertices[vertexIndex]), faceIndex * 9 + corner * 3)
        colors.set([color.r, color.g, color.b], faceIndex * 9 + corner * 3)
      })
    })
    const buffer = new THREE.BufferGeometry()
    buffer.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    buffer.setAttribute('color', new THREE.BufferAttribute(colors, 3))
    buffer.computeVertexNormals()
    return buffer
  }, [data])
  return (
    <mesh geometry={geometry} castShadow receiveShadow>
      <meshStandardMaterial vertexColors side={THREE.DoubleSide} roughness={0.78} metalness={0.02} />
    </mesh>
  )
}

function SmplxMesh({ data, opacity }: { data: PresentationData; opacity: number }) {
  const geometry = useMemo(() => {
    const positions = new Float32Array(data.smplx.faces.length * 9)
    data.smplx.faces.forEach((face, faceIndex) => {
      face.forEach((vertexIndex, corner) => {
        positions.set(rosToThree(data.smplx.vertices[vertexIndex]), faceIndex * 9 + corner * 3)
      })
    })
    const buffer = new THREE.BufferGeometry()
    buffer.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    buffer.computeVertexNormals()
    return buffer
  }, [data])
  return (
    <mesh geometry={geometry} visible={opacity > 0.001}>
      <meshStandardMaterial
        color="#d1d5db"
        transparent
        opacity={opacity}
        side={THREE.DoubleSide}
        roughness={0.68}
        metalness={0.04}
        depthWrite={opacity > 0.92}
      />
    </mesh>
  )
}

function PointCloud({
  points,
  colors,
  opacity,
  size,
  enhanceColors = true,
}: {
  points: Vec3[]
  colors: Vec3[]
  opacity: number
  size: number
  enhanceColors?: boolean
}) {
  const geometry = useMemo(() => {
    const positions = new Float32Array(points.length * 3)
    const colorValues = new Float32Array(points.length * 3)
    points.forEach((point, index) => {
      positions.set(rosToThree(point), index * 3)
      const source = colors[index] ?? [0.55, 0.62, 0.70]
      colorValues.set(
        source.map((channel) => enhanceColors ? Math.min(1, channel * 1.3 + 0.08) : channel),
        index * 3,
      )
    })
    const buffer = new THREE.BufferGeometry()
    buffer.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    buffer.setAttribute('color', new THREE.BufferAttribute(colorValues, 3))
    return buffer
  }, [colors, enhanceColors, points])
  return (
    <points geometry={geometry} visible={opacity > 0.001}>
      <pointsMaterial vertexColors size={size} sizeAttenuation transparent opacity={opacity} depthWrite={false} />
    </points>
  )
}

function SemanticPointCloud({
  points,
  partIds,
  opacity,
}: {
  points: Vec3[]
  partIds: number[]
  opacity: number
}) {
  const colors = useMemo(
    () => partIds.map((partId) => {
      const color = new THREE.Color(PART_COLORS[partId % PART_COLORS.length])
      return [color.r, color.g, color.b] as Vec3
    }),
    [partIds],
  )
  return <PointCloud points={points} colors={colors} opacity={opacity} size={0.051} enhanceColors={false} />
}

function PresentationCamera({
  segment,
  elapsed,
  duration,
}: {
  segment: 'subject' | 'pipeline'
  elapsed: number
  duration: number
}) {
  const { camera } = useThree()
  const target = useRef(new THREE.Vector3(1.35, 0.91, 0))
  useFrame(() => {
    const progress = Math.min(1, elapsed / Math.max(duration, 0.001))
    const angle = segment === 'subject'
      ? -Math.PI * 0.60 + progress * Math.PI * 2
      : -Math.PI * 0.72 + progress * Math.PI * 0.18
    const radius = segment === 'subject' ? 2.45 : 2.30
    camera.position.set(
      target.current.x + Math.cos(angle) * radius,
      segment === 'subject' ? 1.05 : 1.12,
      target.current.z + Math.sin(angle) * radius,
    )
    camera.lookAt(target.current)
  })
  return null
}

function smoothstep(start: number, end: number, value: number) {
  const t = Math.max(0, Math.min(1, (value - start) / Math.max(0.001, end - start)))
  return t * t * (3 - 2 * t)
}
