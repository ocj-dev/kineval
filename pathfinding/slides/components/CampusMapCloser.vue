<script setup lang="ts">
// A coded illustration for the closing slide, in the deck's Google Maps
// theme: a stylized map card of the Diag (the crossing-paths quad at the
// center of the University of Michigan's Ann Arbor campus) with a course
// robot standing at the pin. Built from plain SVG shapes rather than a
// photo/AI-generated image -- there is no image-generation tool available
// in this environment (the sortingthespiderverse deck's closer used AWS
// PartyRock, outside Claude Code).
</script>

<template>
  <svg viewBox="0 0 800 520" class="campus-map" xmlns="http://www.w3.org/2000/svg">
    <!-- map background -->
    <rect x="0" y="0" width="800" height="520" rx="16" fill="#f2efe9" />

    <!-- lawn quadrants split by the Diag's crossing paths -->
    <polygon points="0,0 400,220 0,440" fill="#cdeac0" />
    <polygon points="800,0 400,220 800,440" fill="#cdeac0" />
    <polygon points="0,0 400,220 800,0" fill="#dff0d6" />
    <polygon points="0,440 400,220 800,440" fill="#b9e2a8" />

    <!-- diagonal paths -->
    <polygon points="392,0 408,0 800,432 800,448" fill="#f2efe9" />
    <polygon points="0,432 0,448 408,0 392,0" fill="#f2efe9" />
    <rect x="0" y="204" width="800" height="32" fill="#f2efe9" />

    <!-- trees -->
    <g fill="#5c8a4a">
      <circle cx="80" cy="70" r="16" /><circle cx="120" cy="90" r="12" />
      <circle cx="700" cy="80" r="16" /><circle cx="660" cy="60" r="12" />
      <circle cx="90" cy="380" r="14" /><circle cx="60" cy="410" r="10" />
      <circle cx="710" cy="390" r="14" /><circle cx="740" cy="360" r="10" />
    </g>

    <!-- Angell-Hall-esque colonnade, abstracted, north of the Diag -->
    <g transform="translate(300,40)">
      <rect x="0" y="30" width="200" height="14" fill="#c9c2a6" />
      <rect x="10" y="0" width="12" height="34" fill="#e4ddc2" />
      <rect x="40" y="0" width="12" height="34" fill="#e4ddc2" />
      <rect x="70" y="0" width="12" height="34" fill="#e4ddc2" />
      <rect x="100" y="0" width="12" height="34" fill="#e4ddc2" />
      <rect x="130" y="0" width="12" height="34" fill="#e4ddc2" />
      <rect x="160" y="0" width="12" height="34" fill="#e4ddc2" />
      <polygon points="-10,0 210,0 200,-24 0,-24" fill="#d8d0ac" />
    </g>

    <!-- route line from the edge of the quad up to the pin -->
    <path d="M 60 460 C 220 420, 300 300, 400 250" stroke="#1a73e8" stroke-width="6"
          fill="none" stroke-linecap="round" stroke-dasharray="2 14" />

    <!-- course robot: a small wheeled mobile base with a lidar puck,
         in Michigan maize and blue, parked at the pin -->
    <g transform="translate(400,300)">
      <ellipse cx="0" cy="46" rx="34" ry="8" fill="#00000022" />
      <rect x="-30" y="0" width="60" height="34" rx="10" fill="#00274c" />
      <rect x="-30" y="0" width="60" height="10" rx="5" fill="#ffcb05" />
      <circle cx="-22" cy="36" r="9" fill="#202124" />
      <circle cx="22" cy="36" r="9" fill="#202124" />
      <rect x="-8" y="-16" width="16" height="16" rx="3" fill="#00274c" />
      <circle cx="0" cy="-22" r="7" fill="#ffcb05" stroke="#00274c" stroke-width="2" />
    </g>

    <!-- Google-Maps-style pin marking the Diag -->
    <g transform="translate(400,300)">
      <path d="M0,-58 C-28,-58 -46,-36 -46,-10 C-46,26 0,70 0,70 C0,70 46,26 46,-10 C46,-36 28,-58 0,-58 Z"
            fill="#ea4335" stroke="#a30000" stroke-width="2" opacity="0.92" />
      <circle cx="0" cy="-8" r="16" fill="#fff" />
    </g>

    <!-- info card, like a Google Maps place card -->
    <g transform="translate(560,60)">
      <rect x="0" y="0" width="210" height="92" rx="12" fill="#ffffff" stroke="#e0e0e0" />
      <text x="16" y="30" font-family="Roboto, sans-serif" font-weight="700" font-size="20" fill="#202124">The Diag</text>
      <text x="16" y="52" font-family="Roboto, sans-serif" font-size="13" fill="#5f6368">Central Campus &#183; Ann Arbor, MI</text>
      <text x="16" y="74" font-family="Roboto, sans-serif" font-size="13" fill="#1a73e8">Go Blue! &#128309;</text>
    </g>
  </svg>
</template>

<style scoped>
.campus-map {
  width: 100%;
  height: 100%;
  max-height: 100%;
  border-radius: 12px;
  box-shadow: 0 2px 6px #00000033;
  background: #f2efe9;
}
</style>
