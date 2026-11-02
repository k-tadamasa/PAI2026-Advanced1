import { mkdir, rename } from 'node:fs/promises'
import { dirname, resolve } from 'node:path'
import { spawn } from 'node:child_process'
import { chromium } from 'playwright-core'

const target =
  process.argv[2] ?? 'http://127.0.0.1:5174/?scene=data/private-ai-scene.json'
const output = resolve(
  process.argv[3] ?? '../output/video/PAI最終課題_TADAMASA_NDA.mp4',
)
const poster = resolve(
  process.argv[4] ?? '../output/video/PAI最終課題_TADAMASA_NDA_poster.png',
)
const rawDirectory = resolve('../output/video/.realtime-recording')
const rawVideo = resolve('../output/video/.realtime-recording/raw.webm')
const ffmpeg =
  process.env.FFMPEG_PATH ??
  resolve(
    '../.venv/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2',
  )

await mkdir(rawDirectory, { recursive: true })
await mkdir(dirname(output), { recursive: true })
await mkdir(dirname(poster), { recursive: true })

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
await page.waitForFunction(() => window.__PAI_CAPTURE__?.ready === true, null, {
  timeout: 60_000,
})
const frameCount = await page.evaluate(
  () => window.__PAI_CAPTURE__?.frameCount ?? 0,
)
if (!frameCount) throw new Error('viewer exposed no trajectory frames')

let maximumRenderedPositionErrorM = 0
let maximumRenderedOrientationErrorRad = 0
const trackingSamples = [
  ...new Set([
    0,
    Math.round(frameCount * 0.25),
    Math.round(frameCount * 0.5),
    Math.round(frameCount * 0.75),
    frameCount - 1,
  ]),
]
for (const sample of trackingSamples) {
  await page.evaluate((frame) => window.__PAI_CAPTURE__?.setFrame(frame), sample)
  await page.evaluate(
    () =>
      new Promise((resolveFrame) =>
        requestAnimationFrame(() => requestAnimationFrame(resolveFrame)),
      ),
  )
  const errors = await page.evaluate(() => ({
    position: window.__PAI_CAPTURE__?.renderedDetectorPositionErrorM,
    orientation: window.__PAI_CAPTURE__?.renderedDetectorOrientationErrorRad,
  }))
  if (errors.position == null || errors.orientation == null) {
    throw new Error('viewer exposed no rendered detector tracking diagnostics')
  }
  maximumRenderedPositionErrorM = Math.max(
    maximumRenderedPositionErrorM,
    errors.position,
  )
  maximumRenderedOrientationErrorRad = Math.max(
    maximumRenderedOrientationErrorRad,
    errors.orientation,
  )
}
if (maximumRenderedPositionErrorM > 1e-6) {
  throw new Error(
    `rendered MD position does not track the requested path: ${maximumRenderedPositionErrorM} m`,
  )
}
if (maximumRenderedOrientationErrorRad > (2 * Math.PI) / 180) {
  throw new Error(
    `rendered MD orientation exceeds the relaxed 2 degree limit: ${maximumRenderedOrientationErrorRad} rad`,
  )
}
console.log(
  `rendered tracking max: ${(maximumRenderedPositionErrorM * 1e6).toFixed(3)} um, ` +
    `${((maximumRenderedOrientationErrorRad * 180) / Math.PI).toFixed(6)} deg`,
)

await page.evaluate(() => {
  window.__PAI_CAPTURE__?.setFrame(0)
  window.__PAI_CAPTURE__?.setCameraMode('detector')
})
await page.waitForTimeout(3_000)
const posterFrame = Math.min(frameCount - 1, Math.round(frameCount * 0.48))
await page.evaluate((frame) => {
  window.__PAI_CAPTURE__?.setFrame(frame)
  window.__PAI_CAPTURE__?.setCameraMode('overview')
}, posterFrame)
await page.evaluate(
  () =>
    new Promise((resolveFrame) =>
      requestAnimationFrame(() => requestAnimationFrame(resolveFrame)),
    ),
)
await page.screenshot({ path: poster, type: 'png' })

await page.evaluate(() => {
  window.__PAI_CAPTURE__?.setFrame(0)
  window.__PAI_CAPTURE__?.setCameraMode('detector')
})
await page.evaluate(
  () =>
    new Promise((resolveFrame) =>
      requestAnimationFrame(() => requestAnimationFrame(resolveFrame)),
    ),
)
const playbackStarted = Date.now()
await page.locator('.icon-button--primary').click()
setTimeout(() => {
  page
    .evaluate(() => window.__PAI_CAPTURE__?.setCameraMode('overview'))
    .catch(() => {})
}, 3_600)
await page.waitForFunction(
  (lastFrame) => {
    const timeline = document.querySelector(
      'input[aria-label="軌道再生位置"]',
    )
    return Number(timeline?.value ?? -1) >= lastFrame
  },
  frameCount - 1,
  { timeout: frameCount * 100 + 60_000, polling: 'raf' },
)
await page.evaluate(
  (lastFrame) => window.__PAI_CAPTURE__?.setFrame(lastFrame),
  frameCount - 1,
)
await page.waitForTimeout(400)
console.log(`record ${frameCount}/${frameCount}`)

const video = page.video()
await context.close()
await browser.close()
const recordedPath = await video.path()
await rename(recordedPath, rawVideo)

const prefixSeconds = Math.max(
  0,
  (playbackStarted - recordingStarted) / 1000,
)
console.log(`prefixSeconds=${prefixSeconds.toFixed(3)}`)
await new Promise((resolveProcess, rejectProcess) => {
  const encoder = spawn(
    ffmpeg,
    [
      '-y',
      '-ss',
      prefixSeconds.toFixed(3),
      '-i',
      rawVideo,
      '-c:v',
      'libx264',
      '-preset',
      'medium',
      '-crf',
      '18',
      '-r',
      '30',
      '-pix_fmt',
      'yuv420p',
      '-movflags',
      '+faststart',
      '-metadata',
      'title=PAI最終課題_TADAMASA_NDA',
      '-metadata',
      'artist=TADAMASA_NDA',
      output,
    ],
    { stdio: 'inherit' },
  )
  encoder.on('error', rejectProcess)
  encoder.on('exit', (code) => {
    if (code === 0) resolveProcess()
    else rejectProcess(new Error(`ffmpeg exited with code ${code}`))
  })
})

console.log(output)
console.log(poster)
