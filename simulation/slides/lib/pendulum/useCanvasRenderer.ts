import { onMounted, onBeforeUnmount, type Ref } from 'vue'

// Shared canvas bootstrap for every 2D panel in this deck. A plain
// onMounted()-only measurement (canvas.clientWidth/Height, once) is not
// reliable inside Slidev: a slide's flex layout is not always settled by
// the time its components mount -- most visibly on a cold full-page load
// straight to a slide's URL, where the container can still measure 0x0 at
// mount time, leaving the canvas permanently blank. A ResizeObserver fixes
// this the same way the reference implementation's own main 3D canvas
// already handles resizing (see reference/infrastructure.js's
// observeContainerResize): whenever the canvas's actual box size changes
// (including the very first time it becomes non-zero), re-measure and
// redraw.
export function useCanvasRenderer(
  canvasEl: Ref<HTMLCanvasElement | null>,
  draw: (ctx: CanvasRenderingContext2D, width: number, height: number) => void,
) {
  let ctx: CanvasRenderingContext2D | null = null
  let ro: ResizeObserver | null = null

  function redraw() {
    const canvas = canvasEl.value
    if (!canvas) return
    if (!ctx) ctx = canvas.getContext('2d')
    if (!ctx) return
    const w = canvas.clientWidth, h = canvas.clientHeight
    if (w === 0 || h === 0) return
    if (canvas.width !== w) canvas.width = w
    if (canvas.height !== h) canvas.height = h
    draw(ctx, w, h)
  }

  onMounted(() => {
    redraw()
    const canvas = canvasEl.value
    if (canvas) {
      ro = new ResizeObserver(() => redraw())
      ro.observe(canvas)
    }
  })
  onBeforeUnmount(() => { if (ro) ro.disconnect() })

  return { redraw }
}
