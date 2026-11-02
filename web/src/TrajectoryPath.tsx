import { useEffect, useMemo, useRef } from 'react'
import { Line } from '@react-three/drei'
import type { Line2, LineSegments2 } from 'three-stdlib'
import { rosToThree } from './sceneMath'
import type { SceneData } from './types'

interface TrajectoryPathProps {
  data: SceneData
  frameIndex: number
  showCollisions: boolean
}

export function TrajectoryPath({ data, frameIndex, showCollisions }: TrajectoryPathProps) {
  const traversedRef = useRef<Line2 | LineSegments2>(null)
  const { collisionSegments, points } = useMemo(() => {
    const collisions: [number, number, number][] = []
    const pathPoints = data.trajectory.map((frame) => rosToThree(frame.position_m))
    for (let index = 1; index < pathPoints.length; index += 1) {
      if (data.trajectory[index].collision) {
        collisions.push(pathPoints[index - 1], pathPoints[index])
      }
    }
    return { collisionSegments: collisions, points: pathPoints }
  }, [data])

  useEffect(() => {
    if (!traversedRef.current) return
    traversedRef.current.geometry.instanceCount = Math.max(0, frameIndex)
  }, [frameIndex])

  return (
    <group>
      <Line
        points={points}
        color="#38bdf8"
        lineWidth={2.2}
        transparent
        opacity={0.58}
        renderOrder={1}
      />
      <Line
        ref={traversedRef}
        points={points}
        color="#f8fafc"
        lineWidth={2.2}
        transparent
        opacity={0.98}
        visible={frameIndex > 0}
        renderOrder={2}
      />
      {showCollisions && collisionSegments.length > 1 && (
        <Line
          points={collisionSegments}
          segments
          color="#ef4444"
          lineWidth={2.8}
          transparent
          opacity={0.92}
          renderOrder={3}
        />
      )}
    </group>
  )
}
