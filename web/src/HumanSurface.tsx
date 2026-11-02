import { useEffect, useMemo, useRef } from 'react'
import * as THREE from 'three'
import { colorArray, rosToThree } from './sceneMath'
import type { SceneData } from './types'

const TARGET = new THREE.Color('#b6cce0')
const EXCLUDED = new THREE.Color('#d1d5db')
const INSPECTED = new THREE.Color('#22c55e')
const COLLISION = new THREE.Color('#ef4444')
const NEVER_REACHED = 2_147_483_647

interface HumanSurfaceProps {
  data: SceneData
  frameIndex: number
  showInspected: boolean
  showCollisions: boolean
}

export function HumanSurface({
  data,
  frameIndex,
  showInspected,
  showCollisions,
}: HumanSurfaceProps) {
  const previousFrame = useRef(-1)
  const previousShowInspected = useRef(showInspected)
  const previousShowCollisions = useRef(showCollisions)
  const cumulativeCollisionFaceCount = useRef(0)
  const {
    geometry,
    colors,
    firstInspectionFrame,
    firstCollisionFrame,
    targetFaceSet,
  } = useMemo(() => {
    const faceCount = data.human_mesh.faces.length
    const positions = new Float32Array(faceCount * 9)
    const colorValues = new Float32Array(faceCount * 9)
    const targetFaces = new Set(data.human_mesh.target_face_ids)
    const firstFrame = new Int32Array(faceCount)
    const firstCollision = new Int32Array(faceCount)
    firstFrame.fill(NEVER_REACHED)
    firstCollision.fill(NEVER_REACHED)

    data.trajectory.forEach((frame, index) => {
      frame.newly_inspected_face_ids.forEach((faceId) => {
        if (firstFrame[faceId] === NEVER_REACHED) firstFrame[faceId] = index
      })
      frame.colliding_face_ids.forEach((faceId) => {
        if (firstCollision[faceId] === NEVER_REACHED) firstCollision[faceId] = index
      })
    })

    data.human_mesh.faces.forEach((face, faceIndex) => {
      face.forEach((vertexIndex, corner) => {
        const converted = rosToThree(data.human_mesh.vertices[vertexIndex])
        positions.set(converted, faceIndex * 9 + corner * 3)
      })
      colorArray(targetFaces.has(faceIndex) ? TARGET : EXCLUDED, colorValues, faceIndex * 9)
    })

    const buffer = new THREE.BufferGeometry()
    buffer.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    buffer.setAttribute('color', new THREE.BufferAttribute(colorValues, 3))
    buffer.computeVertexNormals()
    return {
      geometry: buffer,
      colors: colorValues,
      firstInspectionFrame: firstFrame,
      firstCollisionFrame: firstCollision,
      targetFaceSet: targetFaces,
    }
  }, [data])

  useEffect(() => {
    const requiresReset =
      frameIndex < previousFrame.current
      || showInspected !== previousShowInspected.current
      || showCollisions !== previousShowCollisions.current
    if (requiresReset) {
      cumulativeCollisionFaceCount.current = 0
      data.human_mesh.faces.forEach((_, faceIndex) => {
        let color = targetFaceSet.has(faceIndex) ? TARGET : EXCLUDED
        if (showInspected && firstInspectionFrame[faceIndex] <= frameIndex) color = INSPECTED
        if (showCollisions && firstCollisionFrame[faceIndex] <= frameIndex) {
          color = COLLISION
          cumulativeCollisionFaceCount.current += 1
        }
        colorArray(color, colors, faceIndex * 9)
      })
    } else {
      for (let index = previousFrame.current + 1; index <= frameIndex; index += 1) {
        const frame = data.trajectory[index]
        if (!frame) continue
        if (showInspected) {
          frame.newly_inspected_face_ids.forEach((faceIndex) => {
            if (!showCollisions || firstCollisionFrame[faceIndex] > frameIndex) {
              colorArray(INSPECTED, colors, faceIndex * 9)
            }
          })
        }
        if (showCollisions) {
          frame.colliding_face_ids.forEach((faceIndex) => {
            if (firstCollisionFrame[faceIndex] === index) {
              colorArray(COLLISION, colors, faceIndex * 9)
              cumulativeCollisionFaceCount.current += 1
            }
          })
        }
      }
    }
    previousFrame.current = frameIndex
    previousShowInspected.current = showInspected
    previousShowCollisions.current = showCollisions
    if (window.__PAI_CAPTURE__) {
      window.__PAI_CAPTURE__.cumulativeCollisionFaceCount = cumulativeCollisionFaceCount.current
    }
    const attribute = geometry.getAttribute('color') as THREE.BufferAttribute
    attribute.needsUpdate = true
  }, [colors, data, firstCollisionFrame, firstInspectionFrame, frameIndex, geometry, showCollisions, showInspected, targetFaceSet])

  return (
    <group>
      <mesh geometry={geometry} castShadow receiveShadow>
        <meshStandardMaterial
          vertexColors
          side={THREE.DoubleSide}
          roughness={0.72}
          metalness={0.04}
        />
      </mesh>
      {data.human_mesh.faces.length < 5_000 && <GhostMannequin />}
    </group>
  )
}

function GhostMannequin() {
  const material = (
    <meshStandardMaterial color="#d1d5db" transparent opacity={0.22} roughness={0.9} />
  )
  return (
    <group position={[1.2, 0, 0]}>
      <mesh position={[0, 1.65, 0]} scale={[0.11, 0.14, 0.11]}>
        <sphereGeometry args={[1, 32, 20]} />
        {material}
      </mesh>
      <mesh position={[0.08, 1.18, 0]} scale={[0.16, 0.30, 0.27]}>
        <sphereGeometry args={[1, 32, 20]} />
        {material}
      </mesh>
      <mesh position={[0.11, 0.82, 0]} scale={[0.15, 0.13, 0.24]}>
        <sphereGeometry args={[1, 32, 20]} />
        {material}
      </mesh>
      {[-0.15, 0.15].map((z) => (
        <group key={z}>
          <mesh position={[0.06, 0.47, z]} scale={[0.11, 0.28, 0.105]}>
            <sphereGeometry args={[1, 24, 16]} />
            {material}
          </mesh>
          <mesh position={[0.05, 0.03, z]} scale={[0.09, 0.22, 0.09]}>
            <sphereGeometry args={[1, 24, 16]} />
            {material}
          </mesh>
        </group>
      ))}
      {[-0.36, 0.36].map((z) => (
        <mesh key={z} position={[0.05, 1.14, z]} scale={[0.075, 0.34, 0.075]}>
          <sphereGeometry args={[1, 20, 14]} />
          {material}
        </mesh>
      ))}
    </group>
  )
}
