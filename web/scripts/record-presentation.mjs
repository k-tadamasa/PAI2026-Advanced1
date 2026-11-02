import { mkdir, rename } from 'node:fs/promises'
import { basename, dirname, resolve } from 'node:path'
import { spawn } from 'node:child_process'
import { chromium } from 'playwright-core'

const target = process.argv[2]
const output = resolve(process.argv[3])
const poster = process.argv[4] ? resolve(process.argv[4]) : null
if (!target || !output) {
  throw new Error('usage: record-presentation.mjs URL OUTPUT.mp4 [POSTER.png]')
}

const rawDirectory = resolve(dirname(output), '.presentation-recording')
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
await page.waitForFunction(() => window.__PAI_PRESENTATION__?.ready === true, null, { timeout: 60_000 })
const duration = await page.evaluate(() => window.__PAI_PRESENTATION__?.duration ?? 0)
if (!duration) throw new Error('presentation duration is unavailable')
await page.waitForTimeout(800)
if (poster) await page.screenshot({ path: poster, type: 'png' })

const playbackStarted = Date.now()
await page.evaluate(() => window.__PAI_PRESENTATION__?.start())
await page.waitForFunction(() => window.__PAI_PRESENTATION__?.done === true, null, {
  timeout: duration * 1000 + 30_000,
  polling: 'raf',
})
await page.waitForTimeout(250)

const video = page.video()
await context.close()
await browser.close()
const recordedPath = await video.path()
await rename(recordedPath, rawVideo)

const prefixSeconds = Math.max(0, (playbackStarted - recordingStarted) / 1000)
await run(ffmpeg, [
  '-y', '-ss', prefixSeconds.toFixed(3), '-i', rawVideo,
  '-t', duration.toFixed(3), '-an',
  '-vf', 'fps=30,format=yuv420p',
  '-c:v', 'libx264', '-preset', 'medium', '-crf', '18',
  '-movflags', '+faststart',
  '-metadata', 'title=PAI最終課題_TADAMASA_NDA',
  '-metadata', 'artist=TADAMASA_NDA',
  output,
])
console.log(`${output} (${duration.toFixed(1)} s)`)

function run(command, args) {
  return new Promise((resolveProcess, rejectProcess) => {
    const child = spawn(command, args, { stdio: 'inherit' })
    child.on('error', rejectProcess)
    child.on('exit', (code) => code === 0 ? resolveProcess() : rejectProcess(new Error(`${command} exited with ${code}`)))
  })
}
