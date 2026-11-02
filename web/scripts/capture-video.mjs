import { mkdir, rm } from 'node:fs/promises'
import { dirname, resolve } from 'node:path'
import { spawn } from 'node:child_process'
import { chromium } from 'playwright-core'

const target = process.argv[2] ?? 'http://127.0.0.1:5173/?scene=data/private-ai-scene.json'
const output = resolve(
  process.argv[3] ?? '../output/video/PAI最終課題_TADAMASA_NDA.mp4',
)
const poster = resolve(
  process.argv[4] ?? '../output/video/PAI最終課題_TADAMASA_NDA_poster.png',
)
const sourceFps = Number(process.env.SOURCE_FPS ?? 10)
const gripCloseupFrames = Number(process.env.GRIP_CLOSEUP_FRAMES ?? 36)
const frameDirectory = resolve('../output/video/.frames-private-ai')
const ffmpeg =
  process.env.FFMPEG_PATH ??
  resolve('../.venv/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2')

await rm(frameDirectory, { recursive: true, force: true })
await mkdir(frameDirectory, { recursive: true })
await mkdir(dirname(output), { recursive: true })
await mkdir(dirname(poster), { recursive: true })

const browser = await chromium.launch({
  executablePath: process.env.CHROME_PATH ?? '/usr/bin/google-chrome',
  headless: true,
  args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-webgl'],
})
const page = await browser.newPage({
  viewport: { width: 1920, height: 1080 },
  deviceScaleFactor: 1,
})
const captureTarget = new URL(target)
captureTarget.searchParams.set('capture', '1')
const browserErrors = []
page.on('console', (message) => {
  if (message.type() === 'error') browserErrors.push(message.text())
})
page.on('pageerror', (error) => browserErrors.push(error.message))

await page.goto(captureTarget.toString(), { waitUntil: 'networkidle' })
await page.waitForFunction(() => window.__PAI_CAPTURE__?.ready === true, null, {
  timeout: 60_000,
})
await page.evaluate(() => window.__PAI_CAPTURE__?.setCameraMode('detector'))
await page.waitForTimeout(2_000)
const frameCount = await page.evaluate(() => window.__PAI_CAPTURE__?.frameCount ?? 0)
if (!frameCount) throw new Error('viewer exposed no capture frames')

for (let frame = 0; frame < frameCount; frame += 1) {
  if (frame === gripCloseupFrames) {
    await page.evaluate(() => window.__PAI_CAPTURE__?.setCameraMode('overview'))
    await page.waitForTimeout(1_500)
  }
  await page.evaluate((value) => window.__PAI_CAPTURE__?.setFrame(value), frame)
  await page.evaluate(
    () => new Promise((resolveFrame) => requestAnimationFrame(() => requestAnimationFrame(resolveFrame))),
  )
  const filename = `${frameDirectory}/frame-${String(frame).padStart(5, '0')}.jpg`
  await page.screenshot({ path: filename, type: 'jpeg', quality: 92 })
  if (frame % 50 === 0 || frame + 1 === frameCount) {
    console.log(`capture ${frame + 1}/${frameCount}`)
  }
}

const posterFrame = Math.min(
  frameCount - 1,
  Number(process.env.POSTER_FRAME ?? Math.round(frameCount * 0.48)),
)
await page.evaluate((value) => window.__PAI_CAPTURE__?.setFrame(value), posterFrame)
await page.evaluate(
  () => new Promise((resolveFrame) => requestAnimationFrame(() => requestAnimationFrame(resolveFrame))),
)
// Let the browser finish its post-capture flex/layout pass before the lossless
// poster. Without this pause, Chromium can snapshot a transiently compressed
// legend immediately after hundreds of WebGL frame updates.
await page.waitForTimeout(1_000)
await page.screenshot({ path: poster, type: 'png' })
await browser.close()

if (browserErrors.length) {
  throw new Error(`browser errors during capture:\n${browserErrors.join('\n')}`)
}

await new Promise((resolveProcess, rejectProcess) => {
  const encoder = spawn(
    ffmpeg,
    [
      '-y',
      '-framerate',
      String(sourceFps),
      '-i',
      `${frameDirectory}/frame-%05d.jpg`,
      '-c:v',
      'libx264',
      '-preset',
      'slow',
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

if (process.env.KEEP_FRAMES !== '1') {
  await rm(frameDirectory, { recursive: true, force: true })
}
console.log(output)
console.log(poster)
