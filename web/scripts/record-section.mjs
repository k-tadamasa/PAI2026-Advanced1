import { mkdir, rename } from 'node:fs/promises'
import { basename, dirname, resolve } from 'node:path'
import { spawn } from 'node:child_process'
import { chromium } from 'playwright-core'

const target = process.argv[2]
const output = resolve(process.argv[3])
const poster = process.argv[4] ? resolve(process.argv[4]) : null
const cameraMode = process.argv[5] ?? 'overview'
const playbackSpeed = Number(process.argv[6] ?? '2')
if (!target || !output || !['overview', 'detector'].includes(cameraMode)) {
  throw new Error('usage: record-section.mjs URL OUTPUT.mp4 POSTER.png overview|detector [SPEED]')
}

const rawDirectory = resolve(dirname(output), '.section-recording')
const rawVideo = resolve(rawDirectory, `${basename(output, '.mp4')}.webm`)
const ffmpeg = process.env.FFMPEG_PATH ?? resolve('../.venv/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2')
await mkdir(rawDirectory, { recursive: true })
await mkdir(dirname(output), { recursive: true })
if (poster) await mkdir(dirname(poster), { recursive: true })

const browser = await chromium.launch({
  executablePath: process.env.CHROME_PATH ?? '/usr/bin/google-chrome',
  headless: true,
  args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-webgl'],
})
const context = await browser.newContext({
  viewport: { width: 1920, height: 1080 },
  deviceScaleFactor: 1,
  recordVideo: { dir: rawDirectory, size: { width: 1920, height: 1080 } },
})
const recordingStarted = Date.now()
const page = await context.newPage()
await page.goto(target, { waitUntil: 'networkidle' })
await page.waitForFunction(() => window.__PAI_CAPTURE__?.ready === true, null, { timeout: 60_000 })
const frameCount = await page.evaluate(() => window.__PAI_CAPTURE__?.frameCount ?? 0)
if (!frameCount) throw new Error('viewer exposed no trajectory frames')

let maxPositionError = 0
let maxOrientationError = 0
for (const frame of [0, Math.round(frameCount / 2), frameCount - 1]) {
  await page.evaluate((index) => window.__PAI_CAPTURE__?.setFrame(index), frame)
  await page.waitForFunction((index) => {
    const timeline = document.querySelector('input[aria-label="軌道再生位置"]')
    return Number(timeline?.value ?? -1) === index
  }, frame, { timeout: 10_000, polling: 'raf' })
  await page.waitForFunction(() => {
    const position = window.__PAI_CAPTURE__?.renderedDetectorPositionErrorM
    const orientation = window.__PAI_CAPTURE__?.renderedDetectorOrientationErrorRad
    return position != null && orientation != null && position <= 1e-6 && orientation <= (2 * Math.PI) / 180
  }, null, { timeout: 10_000, polling: 'raf' })
  const error = await page.evaluate(() => ({
    position: window.__PAI_CAPTURE__?.renderedDetectorPositionErrorM ?? Number.POSITIVE_INFINITY,
    orientation: window.__PAI_CAPTURE__?.renderedDetectorOrientationErrorRad ?? Number.POSITIVE_INFINITY,
  }))
  maxPositionError = Math.max(maxPositionError, error.position)
  maxOrientationError = Math.max(maxOrientationError, error.orientation)
}
if (maxPositionError > 1e-6 || maxOrientationError > (2 * Math.PI) / 180) {
  throw new Error(`rendered tracking failed: ${maxPositionError} m, ${maxOrientationError} rad`)
}

await page.evaluate(({ mode }) => {
  window.__PAI_CAPTURE__?.setFrame(0)
  window.__PAI_CAPTURE__?.setCameraMode(mode)
}, { mode: cameraMode })
await page.waitForTimeout(1_500)
if (poster) await page.screenshot({ path: poster, type: 'png' })
const playbackStarted = Date.now()
await page.evaluate((speed) => window.__PAI_CAPTURE__?.playAtSpeed(speed), playbackSpeed)
await page.waitForFunction((lastFrame) => {
  const timeline = document.querySelector('input[aria-label="軌道再生位置"]')
  return Number(timeline?.value ?? -1) >= lastFrame
}, frameCount - 1, { timeout: (frameCount * 100) / playbackSpeed + 60_000, polling: 'raf' })
await page.waitForTimeout(300)

const video = page.video()
await context.close()
await browser.close()
const recordedPath = await video.path()
await rename(recordedPath, rawVideo)

const prefixSeconds = Math.max(0, (playbackStarted - recordingStarted) / 1000)
await run(ffmpeg, [
  '-y', '-ss', prefixSeconds.toFixed(3), '-i', rawVideo, '-an',
  '-vf', 'fps=30,format=yuv420p',
  '-c:v', 'libx264', '-preset', 'medium', '-crf', '18',
  '-movflags', '+faststart',
  '-metadata', 'title=PAI最終課題_TADAMASA_NDA',
  '-metadata', 'artist=TADAMASA_NDA',
  output,
])
console.log(`tracking: ${(maxPositionError * 1e6).toFixed(3)} um / ${((maxOrientationError * 180) / Math.PI).toFixed(4)} deg`)
console.log(output)

function run(command, args) {
  return new Promise((resolveProcess, rejectProcess) => {
    const child = spawn(command, args, { stdio: 'inherit' })
    child.on('error', rejectProcess)
    child.on('exit', (code) => code === 0 ? resolveProcess() : rejectProcess(new Error(`${command} exited with ${code}`)))
  })
}
