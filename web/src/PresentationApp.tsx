import { useEffect, useMemo, useRef, useState } from 'react'
import { Activity, ArrowRight, Bot, BrainCircuit, CheckCircle2, Clock3, Route, ScanLine, TimerReset } from 'lucide-react'
import { PresentationScene, type PresentationData } from './PresentationScene'

type Segment = 'intro' | 'subject' | 'pipeline' | 'metrics'

const DURATIONS: Record<Segment, number> = {
  intro: 7,
  subject: 10,
  pipeline: 24,
  metrics: 18,
}

export default function PresentationApp() {
  const query = new URLSearchParams(window.location.search)
  const segment = (query.get('presentation') ?? 'subject') as Segment
  const [data, setData] = useState<PresentationData | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [elapsed, setElapsed] = useState(0)
  const startedAt = useRef<number | null>(null)
  const duration = DURATIONS[segment] ?? DURATIONS.subject

  useEffect(() => {
    fetch(query.get('data') ?? 'data/private-presentation.json')
      .then((response) => {
        if (!response.ok) throw new Error(`HTTP ${response.status}`)
        return response.json() as Promise<PresentationData>
      })
      .then(setData)
      .catch((reason) => setError(String(reason)))
  }, [])

  useEffect(() => {
    if (!data) return
    window.__PAI_PRESENTATION__ = {
      ready: true,
      duration,
      done: false,
      start: () => {
        startedAt.current = performance.now()
        setElapsed(0)
        if (window.__PAI_PRESENTATION__) window.__PAI_PRESENTATION__.done = false
      },
    }
    return () => { delete window.__PAI_PRESENTATION__ }
  }, [data, duration])

  useEffect(() => {
    let request = 0
    const tick = (now: number) => {
      if (startedAt.current == null) {
        request = requestAnimationFrame(tick)
        return
      }
      const next = Math.min(duration, (now - startedAt.current) / 1000)
      setElapsed(next)
      if (next >= duration) {
        if (window.__PAI_PRESENTATION__) window.__PAI_PRESENTATION__.done = true
        return
      }
      request = requestAnimationFrame(tick)
    }
    request = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(request)
  }, [duration])

  if (error) return <main className="presentation-state">{error}</main>
  if (!data) return <main className="presentation-state"><Activity className="spin" /> Loading presentation data</main>

  return (
    <main className={`presentation presentation--${segment}`}>
      {segment === 'intro'
        ? <IntroSegment elapsed={elapsed} />
        : segment === 'metrics'
          ? <MetricsSegment data={data} elapsed={elapsed} />
          : <VisualSegment data={data} segment={segment} elapsed={elapsed} duration={duration} />}
    </main>
  )
}

function IntroSegment({ elapsed }: { elapsed: number }) {
  const purposeProgress = easeOut(clamp((elapsed - 0.35) / 0.7))
  const steps = [
    { icon: <BrainCircuit />, index: '01', label: '人の形状推定', detail: 'Human3D + SMPL-X' },
    { icon: <Route />, index: '02', label: '軌道生成', detail: 'Surface raster + TORM' },
    { icon: <Bot />, index: '03', label: 'シミュレーションでの実行', detail: 'ffw_sg2_mecanum' },
  ]
  return (
    <div className="intro-stage">
      <header className="intro-heading">
        <p>01 / PROJECT OVERVIEW</p>
        <span>PAI FINAL PROJECT · TADAMASA_NDA</span>
      </header>
      <section
        className="intro-purpose"
        style={{ opacity: purposeProgress, transform: `translateY(${(1 - purposeProgress) * 22}px)` }}
      >
        <div><ScanLine size={28} /><span>PURPOSE / 目的</span></div>
        <h1>ロボットによる<br />ボディチェックを行う</h1>
      </section>
      <section className="intro-implementation">
        <p>IMPLEMENTATION / 実装内容</p>
        <div className="intro-flow">
          {steps.map((step, index) => {
            const progress = easeOut(clamp((elapsed - 1.25 - index * 0.65) / 0.75))
            return (
              <div className="intro-flow-item" key={step.index}>
                <article style={{ opacity: progress, transform: `translateX(${(1 - progress) * 26}px)` }}>
                  <div className="intro-flow-icon">{step.icon}</div>
                  <span>{step.index}</span>
                  <strong>{step.label}</strong>
                  <small>{step.detail}</small>
                </article>
                {index < steps.length - 1 && (
                  <ArrowRight
                    className="intro-flow-arrow"
                    size={28}
                    style={{ opacity: easeOut(clamp((elapsed - 1.8 - index * 0.65) / 0.45)) }}
                  />
                )}
              </div>
            )
          })}
        </div>
      </section>
    </div>
  )
}

function VisualSegment({
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
  const stage = segment === 'subject' ? 0 : elapsed < 7 ? 0 : elapsed < 15 ? 1 : 2
  const stages = [
    { label: '3D POINT CLOUD', detail: `${data.acquisition.point_count.toLocaleString()} points` },
    { label: 'HUMAN3D', detail: `${data.human3d.point_count.toLocaleString()} semantic points` },
    { label: 'SEGFIT / SMPL-X', detail: `${data.smplx.vertices.length.toLocaleString()} vertices` },
  ]
  return (
    <>
      <div className="presentation-canvas">
        <PresentationScene data={data} segment={segment} elapsed={elapsed} duration={duration} />
      </div>
      <header className="presentation-header">
        <div>
          <p>{segment === 'subject' ? '02 / SYNTHETIC SUBJECT' : '03 / AI BODY ESTIMATION'}</p>
          <h1>{segment === 'subject' ? '姿勢推定前の対象人物' : '点群から身体表面モデルへ'}</h1>
        </div>
        <span>PAI FINAL PROJECT · TADAMASA_NDA</span>
      </header>
      {segment === 'subject' ? (
        <div className="subject-caption">
          <strong>petite_female</strong>
          <span>Isaac Sim synthetic human · rendered garments · pre-estimation ground truth</span>
        </div>
      ) : (
        <>
          <div className="pipeline-status">
            <span>STEP {stage + 1} / 3</span>
            <strong>{stages[stage].label}</strong>
            <small>{stages[stage].detail}</small>
          </div>
          <div className="pipeline-timeline">
            {stages.map((item, index) => (
              <div className={index <= stage ? 'active' : ''} key={item.label}>
                <i>{index < stage ? <CheckCircle2 size={18} /> : index + 1}</i>
                <span><strong>{item.label}</strong><small>{item.detail}</small></span>
              </div>
            ))}
            <div className="pipeline-progress" style={{ width: `${(elapsed / duration) * 100}%` }} />
          </div>
        </>
      )}
    </>
  )
}

function MetricsSegment({ data, elapsed }: { data: PresentationData; elapsed: number }) {
  const metrics = useMemo(() => [
    { icon: <ScanLine />, label: '検査対象範囲の網羅率', value: data.metrics.coverage_ratio * 100, suffix: '%', digits: 2 },
    { icon: <Clock3 />, label: '検査時間', value: data.metrics.inspection_time_s, suffix: ' s', digits: 1 },
    { icon: <Route />, label: '軌道長', value: data.metrics.path_length_m, suffix: ' m', digits: 2 },
    { icon: <TimerReset />, label: '軌道計算時間', value: data.metrics.planning_time_s, suffix: ' s', digits: 3 },
    { icon: <BrainCircuit />, label: '姿勢推定時間', value: data.metrics.pose_estimation_time_s, suffix: ' s', digits: 3 },
  ], [data])
  return (
    <div className="metrics-stage">
      <header className="metrics-heading">
        <p>06 / ANALYSIS RESULTS</p>
        <h1>身体検査シミュレーション結果</h1>
        <span>Human3D + SegFit / Kitahara surface raster / TORM path-wise IK</span>
      </header>
      <section className="result-grid">
        {metrics.map((metric, index) => {
          const progress = clamp((elapsed - 1.0 - index * 1.15) / 1.8)
          return (
            <article
              className="result-card"
              key={metric.label}
              style={{ opacity: progress, transform: `translateY(${(1 - progress) * 28}px)` }}
            >
              <div>{metric.icon}<span>{metric.label}</span></div>
              <strong>{(metric.value * easeOut(progress)).toFixed(metric.digits)}<small>{metric.suffix}</small></strong>
            </article>
          )
        })}
      </section>
      <footer className="metrics-footer" style={{ opacity: clamp((elapsed - 8.4) / 1.8) }}>
        <div><span>ROBOT</span><strong>ffw_sg2_mecanum · scan speed {data.metrics.scan_speed_scale.toFixed(0)}×</strong></div>
        <div><span>COVERAGE DENOMINATOR</span><strong>Inspection target ROI only</strong></div>
        <div><span>COLLISION</span><strong>{data.metrics.collision_pose_count} poses · avoidance is future work</strong></div>
      </footer>
    </div>
  )
}

function clamp(value: number) {
  return Math.max(0, Math.min(1, value))
}

function easeOut(value: number) {
  return 1 - (1 - value) ** 3
}
