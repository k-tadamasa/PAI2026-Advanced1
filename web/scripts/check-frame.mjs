import { mkdir } from 'node:fs/promises'
import { dirname, resolve } from 'node:path'
import { chromium } from 'playwright-core'

const target = process.argv[2] ?? 'http://127.0.0.1:5173/?scene=data/demo-scene.json'
const frame = Number(process.argv[3] ?? 0)
const output = resolve(process.argv[4] ?? '../output/video/collision-frame-check.png')

await mkdir(dirname(output), { recursive: true })
const browser = await chromium.launch({
  executablePath: process.env.CHROME_PATH ?? '/usr/bin/google-chrome',
  headless: true,
  args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-webgl'],
})
const page = await browser.newPage({ viewport: { width: 1920, height: 1080 } })
const browserErrors = []
page.on('console', (message) => {
  if (message.type() === 'error') browserErrors.push(message.text())
})
page.on('pageerror', (error) => browserErrors.push(error.message))

await page.goto(target, { waitUntil: 'networkidle' })
await page.waitForFunction(() => window.__PAI_CAPTURE__?.ready === true, null, {
  timeout: 60_000,
})
await page.evaluate((value) => window.__PAI_CAPTURE__?.setFrame(value), frame)
await page.waitForFunction(
  () => window.__PAI_CAPTURE__?.collisionComponentColors != null,
  null,
  { timeout: 10_000 },
)
await page.evaluate(
  () => new Promise((resolveFrame) => requestAnimationFrame(() => requestAnimationFrame(resolveFrame))),
)

const result = await page.evaluate(() => ({
  badge: document.querySelector('.collision-badge')?.textContent?.trim(),
  colors: window.__PAI_CAPTURE__?.collisionComponentColors,
  rightGripperLinkNames: window.__PAI_CAPTURE__?.rightGripperLinkNames,
  cumulativeCollisionFaceCount: window.__PAI_CAPTURE__?.cumulativeCollisionFaceCount,
}))
await page.screenshot({ path: output, type: 'png' })
await browser.close()

if (browserErrors.length) {
  throw new Error(`browser errors:\n${browserErrors.join('\n')}`)
}
console.log(JSON.stringify({ frame, output, ...result }, null, 2))
