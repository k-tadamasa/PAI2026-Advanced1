import * as THREE from 'three'
import type { Vec3, Vec4 } from './types'

export function rosToThree([x, y, z]: Vec3): [number, number, number] {
  return [x, z, -y]
}

export function detectorQuaternion([w, x, y, z]: Vec4): THREE.Quaternion {
  const rosQuaternion = new THREE.Quaternion(x, y, z, w).normalize()
  const mappedAxis = (axis: THREE.Vector3) => {
    const rotated = axis.applyQuaternion(rosQuaternion)
    return new THREE.Vector3(...rosToThree([rotated.x, rotated.y, rotated.z]))
  }
  const localX = mappedAxis(new THREE.Vector3(1, 0, 0))
  const localY = mappedAxis(new THREE.Vector3(0, 1, 0))
  const localZ = mappedAxis(new THREE.Vector3(0, 0, 1))
  const matrix = new THREE.Matrix4().makeBasis(localX, localY, localZ)
  return new THREE.Quaternion().setFromRotationMatrix(matrix)
}

export function colorArray(color: THREE.Color, target: Float32Array, offset: number) {
  for (let vertex = 0; vertex < 3; vertex += 1) {
    const index = offset + vertex * 3
    target[index] = color.r
    target[index + 1] = color.g
    target[index + 2] = color.b
  }
}
