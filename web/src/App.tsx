import { useEffect, useRef, useState } from 'react'
import {
  Activity,
  Bot,
  Box,
  Camera,
  CheckCircle2,
  CirclePause,
  CirclePlay,
  Crosshair,
  Eye,
  EyeOff,
  Gauge,
  Maximize2,
  RotateCcw,
  Route,
  ShieldAlert,
  Sparkles,
} from 'lucide-react'
import { ScreeningScene } from './ScreeningScene'
import type { CameraMode, LayerVisibility } from './types'
import { useSceneData } from './useSceneData'

const DEFAULT_LAYERS: LayerVisibility = {
  robot: true,
  detector: true,
  detectionVolume: true,
  inspected: true,
  trajectory: true,
  collisions: true,
  labels: true,
}

function App() {
  const query = new URLSearchParams(window.location.search)
  const captureMode = query.get('capture') === '1'
  const captureView = query.get('view') === 'detector' ? 'detector' : 'overview'
  const captureSpeed = Number(query.get('speed') ?? '1')
  const { data, error } = useSceneData()
  const [frameIndex, setFrameIndex] = useState(0)
  const [playing, setPlaying] = useState(true)
  const [speed, setSpeed] = useState(1)
  const [layers, setLayers] = useState(DEFAULT_LAYERS)
  const [cameraMode, setCameraMode] = useState<CameraMode>(captureView)
  const frameRef = useRef(frameIndex)
  const playbackTimeRef = useRef(0)
  const captureControlledRef = useRef(false)

  useEffect(() => {
    frameRef.current = frameIndex
  }, [frameIndex])

  useEffect(() => {
    if (!data || !playing) return
    let request = 0
    let previous = performance.now()
    const tick = (now: number) => {
      if (captureControlledRef.current) return
      playbackTimeRef.current += ((now - previous) / 1000) * speed
      previous = now
      const lastFrame = data.trajectory.length - 1
      const lastTime = data.trajectory[lastFrame].robot.time_s
      if (playbackTimeRef.current >= lastTime) {
        playbackTimeRef.current = lastTime
        setFrameIndex(lastFrame)
        setPlaying(false)
        return
      }
      let low = frameRef.current
      let high = lastFrame
      if (data.trajectory[low].robot.time_s > playbackTimeRef.current) low = 0
      while (low < high) {
        const middle = Math.ceil((low + high) / 2)
        if (data.trajectory[middle].robot.time_s <= playbackTimeRef.current) {
          low = middle
        } else {
          high = middle - 1
        }
      }
      if (low !== frameRef.current) setFrameIndex(low)
      request = requestAnimationFrame(tick)
    }
    request = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(request)
  }, [data, playing, speed])

  useEffect(() => {
    if (!data) return
    window.__PAI_CAPTURE__ = {
      ready: true,
      frameCount: data.trajectory.length,
      setFrame: (frame: number) => {
        captureControlledRef.current = true
        setPlaying(false)
        const nextFrame = Math.max(0, Math.min(data.trajectory.length - 1, frame))
        playbackTimeRef.current = data.trajectory[nextFrame].robot.time_s
        setFrameIndex(nextFrame)
      },
      setCameraMode,
      playAtSpeed: (requestedSpeed: number) => {
        captureControlledRef.current = false
        playbackTimeRef.current = data.trajectory[frameRef.current].robot.time_s
        setSpeed(requestedSpeed)
        setPlaying(true)
      },
    }
    return () => {
      delete window.__PAI_CAPTURE__
    }
  }, [data])

  if (error) {
    return (
      <main className="state-page" role="alert">
        <ShieldAlert size={36} />
        <h1>デモデータを読み込めませんでした</h1>
        <p>{error}</p>
      </main>
    )
  }
  if (!data) {
    return (
      <main className="state-page" role="status">
        <Activity className="spin" size={36} />
        <h1>PAI Body Screening Viewer</h1>
        <p>公開デモデータを準備しています</p>
      </main>
    )
  }

  const frame = data.trajectory[frameIndex]
  const isLocalAiResult = data.human_mesh.faces.length >= 5_000
  const progress = (frameIndex / Math.max(1, data.trajectory.length - 1)) * 100
  const elapsed = frame.robot.time_s
  const total = data.trajectory[data.trajectory.length - 1].robot.time_s
  const detectorCollision = frame.collision_components.includes('metal_detector_link')
  const gripperCollision = frame.collision_components.some((component) => component.startsWith('gripper_r_'))
  const collisionLabel = frame.collision
    ? `MESH COLLISION · ${[detectorCollision ? 'MD' : '', gripperCollision ? 'GRIPPER' : ''].filter(Boolean).join(' + ')}`
    : 'CLEAR'

  const toggleLayer = (key: keyof LayerVisibility) => {
    setLayers((current) => ({ ...current, [key]: !current[key] }))
  }

  return (
    <main className={`app-shell ${captureMode ? 'app-shell--capture' : ''}`}>
      <header className="topbar">
        <div className="brand">
          <div className="brand-mark" aria-hidden="true"><Crosshair size={22} /></div>
          <div>
            <p className="eyebrow">PHYSICAL AI · ADVANCED 1</p>
            <h1>Kitahara Surface Raster Viewer</h1>
          </div>
        </div>
        <div className="topbar-meta">
          <span className="status-pill"><span className="status-dot" />{isLocalAiResult ? 'LOCAL AI RESULT' : 'PUBLIC DEMO'}</span>
          <span className="account">{data.account.name} / ID {data.account.id}</span>
        </div>
      </header>

      <section className="workspace">
        <aside className="panel panel--left" aria-label="表示設定">
          <div className="panel-heading">
            <div><p className="eyebrow">SCENE</p><h2>表示レイヤー</h2></div>
            <Box size={18} />
          </div>
          <div className="layer-list">
            <LayerButton label="FFW-SG2 Mecanum" active={layers.robot} onClick={() => toggleLayer('robot')} />
            <LayerButton label="MD実体" active={layers.detector} color="#9ca3af" onClick={() => toggleLayer('detector')} />
            <LayerButton label="探知可能領域" active={layers.detectionVolume} color="#3b82f6" onClick={() => toggleLayer('detectionVolume')} />
            <LayerButton label="検査済み領域" active={layers.inspected} color="#22c55e" onClick={() => toggleLayer('inspected')} />
            <LayerButton label={`${data.parameters.lane_count}列往復軌道`} active={layers.trajectory} color="#38bdf8" onClick={() => toggleLayer('trajectory')} />
            <LayerButton label="実形状衝突" active={layers.collisions} color="#ef4444" onClick={() => toggleLayer('collisions')} />
            <LayerButton label="注釈" active={layers.labels} onClick={() => toggleLayer('labels')} />
          </div>

          <div className="divider" />
          <p className="section-label">CAMERA</p>
          <div className="segmented segmented--vertical">
            <CameraButton label="全景" active={cameraMode === 'overview'} onClick={() => setCameraMode('overview')} icon={<Maximize2 size={16} />} />
            <CameraButton label="正面" active={cameraMode === 'front'} onClick={() => setCameraMode('front')} icon={<Camera size={16} />} />
            <CameraButton label="MD追従" active={cameraMode === 'detector'} onClick={() => setCameraMode('detector')} icon={<Crosshair size={16} />} />
          </div>

          <div className="method-card">
            <Sparkles size={17} />
            <div><strong>AIを計画入力に使用</strong><span>Human3D部位推定 + SegFit表面</span></div>
          </div>
          <p className="privacy-note">
            {isLocalAiResult
              ? 'Human3D + SegFitのローカル推論結果です。メッシュ自体はGitHubへ配布しません。'
              : '公開画面は匿名の合成人体面です。実測AIメッシュは配布しません。'}
          </p>
        </aside>

        <section className="viewport-card" aria-label="3D検査軌道ビュー">
          <ScreeningScene
            data={data}
            frameIndex={frameIndex}
            layers={layers}
            cameraMode={cameraMode}
          />
          <div className="viewport-overlay viewport-overlay--top">
            <div className="scene-title">
              <span>ROBOT</span>
              <strong>ffw_sg2_mecanum</strong>
              <small>ideal holonomic base motion</small>
            </div>
            <div className={`collision-badge ${frame.collision ? 'collision-badge--active' : ''}`}>
              <span className="collision-indicator" />
              {collisionLabel}
            </div>
          </div>
          {layers.labels && (
            <div className="axis-key" aria-label="色の凡例">
              <Legend color="#b6cce0" label="検査対象（未検査）" />
              <Legend color="#9ca3af" label="MD実体" />
              <Legend color="#22c55e" label="検査済み" />
              <Legend color="#3b82f6" label="探知領域" />
              <Legend color="#38bdf8" label="未通過軌道" />
              <Legend color="#f8fafc" label="通過済み軌道" />
              <Legend color="#ef4444" label="衝突履歴" />
            </div>
          )}
          {captureMode && captureSpeed > 1 && (
            <div className="capture-speed-badge">
              {captureSpeed.toFixed(0)}× VIDEO · {(data.parameters.scan_speed_scale ?? 1).toFixed(0)}× SCAN
            </div>
          )}
          {captureMode && (
            <div className="capture-chapter-badge">
              {captureView === 'detector' ? '05 / DETECTOR CLOSE-UP' : '04 / ROBOT INSPECTION · OVERVIEW'}
            </div>
          )}
        </section>

        <aside className="panel panel--right" aria-label="検査指標">
          <div className="panel-heading">
            <div><p className="eyebrow">LIVE METRICS</p><h2>検査進捗</h2></div>
            <Gauge size={18} />
          </div>
          <div className="coverage-ring" style={{ '--coverage': `${frame.coverage_ratio * 360}deg` } as React.CSSProperties}>
            <div><strong>{(frame.coverage_ratio * 100).toFixed(1)}%</strong><span>検査対象範囲のみ</span></div>
          </div>
          <div className="metric-grid">
            <Metric icon={<Route size={16} />} label="経路長" value={`${data.metrics.path_length_m.toFixed(2)} m`} />
            <Metric icon={<Activity size={16} />} label="姿勢" value={`${frameIndex + 1} / ${data.metrics.trajectory_pose_count}`} />
            <Metric icon={<Bot size={16} />} label="台車 X / Y" value={`${frame.robot.base_position_m[0].toFixed(2)} / ${frame.robot.base_position_m[1].toFixed(2)}`} />
            <Metric icon={<ShieldAlert size={16} />} label="実形状衝突" value={`${data.metrics.collision_pose_count}`} danger />
          </div>

          <div className="divider" />
          <div className="parameter-list">
            <Parameter label="対象部位ID" value={data.scope.human3d_part_ids.join(', ')} />
            <Parameter label="軌道実装" value={`Kitahara SI ${data.parameters.lane_count}列往復`} />
            <Parameter label="走査列" value={`${data.parameters.lane_count}列 / LOESS ${data.parameters.loess_window_points}点`} />
            <Parameter label="基準走査線間隔" value={`${data.parameters.lane_spacing_scale.toFixed(1)}×`} />
            <Parameter label="中央列間隔" value={`${data.parameters.central_lane_gap_scale.toFixed(1)}× baseline`} />
            <Parameter label="中央MD反転" value={data.parameters.flip_md_at_center ? '180°' : 'OFF'} />
            <Parameter label="IK" value="TORM path-wise 11-DoF" />
            <Parameter label="正面法線" value={`< ${data.parameters.front_normal_max_angle_deg.toFixed(0)}° vs -X`} />
            <Parameter label="左右範囲" value={`中央 ${(data.parameters.front_width_fraction * 100).toFixed(0)}%`} />
            <Parameter label="上端除外" value={`${(data.parameters.front_top_margin_m * 1000).toFixed(0)} mm`} />
            <Parameter label="姿勢刻み" value={`≤ ${data.parameters.maximum_orientation_step_deg.toFixed(1)}°`} />
            <Parameter label="走査速度" value={`${(data.parameters.scan_speed_scale ?? 1).toFixed(1)}×`} />
            <Parameter label="最大制御位置刻み" value={`≤ ${((data.parameters.maximum_control_position_step_m ?? 0.002) * 1000).toFixed(1)} mm`} />
            <Parameter label="最大関節刻み" value={`≤ ${(data.parameters.maximum_control_joint_step_deg ?? 0.5).toFixed(1)}°`} />
            <Parameter label="MD離隔" value={`${(data.parameters.standoff_m * 1000).toFixed(0)} mm`} />
            <Parameter label="探知楕円体" value="110 × 111 × 90.6 mm" />
            <Parameter label="衝突形状" value="MD + 右グリッパ STL" />
            <Parameter label="台車モデル" value="理想Mecanum" />
          </div>

          <div className="future-work">
            <ShieldAlert size={18} />
            <div><strong>Collision avoidance: future work</strong><span>MD・右グリッパ実メッシュの接触を事後判定し、計画は変更しません。</span></div>
          </div>
        </aside>
      </section>

      <footer className="timeline-panel">
        <div className="transport">
          <button
            className="icon-button icon-button--primary"
            onClick={() => {
              playbackTimeRef.current = data.trajectory[frameRef.current].robot.time_s
              setPlaying((current) => !current)
            }}
            aria-label={playing ? '一時停止' : '再生'}
          >
            {playing ? <CirclePause size={22} /> : <CirclePlay size={22} />}
          </button>
          <button className="icon-button" onClick={() => { setPlaying(false); playbackTimeRef.current = 0; setFrameIndex(0) }} aria-label="先頭へ戻る">
            <RotateCcw size={18} />
          </button>
          <span className="timecode">{formatTime(elapsed)} <em>/</em> {formatTime(total)}</span>
        </div>
        <div className="timeline-track">
          <input
            type="range"
            min={0}
            max={data.trajectory.length - 1}
            value={frameIndex}
            onChange={(event) => {
              const nextFrame = Number(event.target.value)
              setPlaying(false)
              playbackTimeRef.current = data.trajectory[nextFrame].robot.time_s
              setFrameIndex(nextFrame)
            }}
            aria-label="軌道再生位置"
            style={{ '--progress': `${progress}%` } as React.CSSProperties}
          />
          <div className="timeline-labels"><span>START</span><span>FRONT ROI SCAN</span><span>RESULT</span></div>
        </div>
        <div className="speed-control" aria-label="再生速度">
          {[0.5, 1, 2].map((value) => (
            <button key={value} className={speed === value ? 'active' : ''} onClick={() => setSpeed(value)} aria-pressed={speed === value}>{value}×</button>
          ))}
        </div>
      </footer>
    </main>
  )
}

function LayerButton({ label, active, color = '#94a3b8', onClick }: { label: string; active: boolean; color?: string; onClick: () => void }) {
  return (
    <button className={`layer-button ${active ? 'active' : ''}`} onClick={onClick} aria-pressed={active}>
      <span className="layer-swatch" style={{ backgroundColor: color }} />
      <span>{label}</span>
      {active ? <Eye size={16} /> : <EyeOff size={16} />}
    </button>
  )
}

function CameraButton({ label, active, onClick, icon }: { label: string; active: boolean; onClick: () => void; icon: React.ReactNode }) {
  return <button className={active ? 'active' : ''} onClick={onClick} aria-pressed={active}>{icon}{label}</button>
}

function Metric({ icon, label, value, danger = false }: { icon: React.ReactNode; label: string; value: string; danger?: boolean }) {
  return <div className={`metric-card ${danger ? 'metric-card--danger' : ''}`}><span>{icon}{label}</span><strong>{value}</strong></div>
}

function Parameter({ label, value }: { label: string; value: string }) {
  return <div><span>{label}</span><strong>{value}</strong></div>
}

function Legend({ color, label }: { color: string; label: string }) {
  return <span><i style={{ backgroundColor: color }} />{label}</span>
}

function formatTime(seconds: number) {
  const minutes = Math.floor(seconds / 60)
  const remaining = Math.floor(seconds % 60)
  return `${String(minutes).padStart(2, '0')}:${String(remaining).padStart(2, '0')}`
}

export default App
