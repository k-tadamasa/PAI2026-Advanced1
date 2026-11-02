import { mkdir } from 'node:fs/promises'
import { dirname, resolve } from 'node:path'
import { chromium } from 'playwright-core'

const target = process.argv[2] ?? 'http://127.0.0.1:5173/'
const output = resolve(process.argv[3] ?? '../output/web/viewer-preview.png')
const width = Number(process.argv[4] ?? 1440)
const height = Number(process.argv[5] ?? 900)
const frame = process.argv[6] === undefined ? null : Number(process.argv[6])
const cameraMode = process.argv[7] ?? null
await mkdir(dirname(output), { recursive: true })

const browser = await chromium.launch({
  executablePath: process.env.CHROME_PATH ?? '/usr/bin/google-chrome',
  headless: true,
  args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-webgl'],
})
const page = await browser.newPage({ viewport: { width, height }, deviceScaleFactor: 1 })
const messages = []
page.on('console', (message) => {
  if (message.type() === 'error' || message.type() === 'warning') messages.push(`${message.type()}: ${message.text()}`)
})
page.on('pageerror', (error) => messages.push(`pageerror: ${error.message}`))
await page.goto(target, { waitUntil: 'networkidle' })
await page.waitForFunction(() => window.__PAI_CAPTURE__?.ready === true, null, { timeout: 60_000 })
if (frame !== null) {
  await page.evaluate((value) => window.__PAI_CAPTURE__?.setFrame(value), frame)
}
if (cameraMode !== null) {
  await page.evaluate((value) => window.__PAI_CAPTURE__?.setCameraMode(value), cameraMode)
}
await page.waitForTimeout(5_000)
await page.screenshot({ path: output, type: 'png' })
console.log(output)
if (messages.length) console.log(messages.join('\n'))
await browser.close()
