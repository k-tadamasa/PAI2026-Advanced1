import { useEffect, useState } from 'react'
import type { SceneData } from './types'

export function useSceneData() {
  const [data, setData] = useState<SceneData | null>(null)
  const [error, setError] = useState<string | null>(null)

  useEffect(() => {
    const controller = new AbortController()
    const requested = new URLSearchParams(window.location.search).get('scene')
    const scenePath = (requested ?? 'data/demo-scene.json').replace(/^\/+/, '')
    fetch(`${import.meta.env.BASE_URL}${scenePath}`, {
      signal: controller.signal,
    })
      .then((response) => {
        if (!response.ok) throw new Error(`scene load failed: ${response.status}`)
        return response.json() as Promise<SceneData>
      })
      .then(setData)
      .catch((reason: unknown) => {
        if ((reason as Error).name !== 'AbortError') {
          setError(reason instanceof Error ? reason.message : String(reason))
        }
      })
    return () => controller.abort()
  }, [])

  return { data, error }
}
