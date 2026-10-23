import { defineConfig } from 'vite'

// The npm-resolved Vite/lightningcss versions as of this deck's build (not
// pinned by any package-lock in this repo, including this one, which only
// locks @slidev/cli/vue -- lightningcss comes along as Vite's own default
// CSS minifier) choke on @slidev/client's own built-in katex.css, unrelated
// to any content in this deck. esbuild isn't available as a fallback
// minifier under this Vite/rolldown build either, so CSS minification is
// just disabled outright -- a larger but still entirely valid stylesheet.
export default defineConfig({
  build: {
    cssMinify: false,
  },
})
