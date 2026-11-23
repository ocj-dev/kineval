// Tree layout for the SVG tree drawings: leaves get evenly spaced slots
// left to right, every internal node is centred over its children, and
// depth sets the row. The classic tidy-tree layout, which is enough here
// because these trees are small and shallow.
//
// Layout is a function of STRUCTURE only. Status never enters it, so a
// tree cannot shift on screen as it runs -- the drawing is computed once
// and only recoloured. That separation is the same one
// ../../../reference/infrastructure.js#tree-ascii uses.
import type { BtNode, NodeKind } from './types'
import { childPath } from './behaviorTree'

export interface LaidOutNode {
  node: BtNode
  path: string
  kind: NodeKind
  name: string
  depth: number
  x: number
  y: number
  parentPath: string | null
}

export interface TreeLayout {
  nodes: LaidOutNode[]
  width: number
  height: number
  byPath: Record<string, LaidOutNode>
}

export function layoutTree(
  root: BtNode,
  options?: { columnWidth?: number; rowHeight?: number; marginX?: number; marginY?: number },
): TreeLayout {
  const columnWidth = options?.columnWidth ?? 150
  const rowHeight = options?.rowHeight ?? 64
  const marginX = options?.marginX ?? 12
  const marginY = options?.marginY ?? 16

  const nodes: LaidOutNode[] = []
  let nextLeafSlot = 0

  function visit(node: BtNode, path: string, depth: number, parentPath: string | null): number {
    const entry: LaidOutNode = {
      node, path, kind: node.kind, name: node.name, depth,
      x: 0, y: marginY + depth * rowHeight, parentPath,
    }
    nodes.push(entry)

    if (!node.children.length) {
      entry.x = marginX + nextLeafSlot * columnWidth + columnWidth / 2
      nextLeafSlot++
      return entry.x
    }

    const childCentres = node.children.map((child, i) => visit(child, childPath(path, i), depth + 1, path))
    entry.x = (childCentres[0] + childCentres[childCentres.length - 1]) / 2
    return entry.x
  }

  visit(root, '', 0, null)

  const maxDepth = nodes.reduce((m, n) => Math.max(m, n.depth), 0)
  const byPath: Record<string, LaidOutNode> = {}
  for (const n of nodes) byPath[n.path] = n

  return {
    nodes,
    width: marginX * 2 + nextLeafSlot * columnWidth,
    height: marginY * 2 + maxDepth * rowHeight + 34,
    byPath,
  }
}

// The glyph each node kind is drawn with, following the conventions the
// behavior-tree literature uses: an arrow for a sequence, a question mark
// for a fallback, a double arrow for a parallel, a diamond for a
// decorator, an oval for a condition and a rectangle for an action.
export const KIND_GLYPH: Record<NodeKind, string> = {
  sequence: '→',
  fallback: '?',
  parallel: '⇉',
  decorator: '◇',
  condition: '',
  action: '',
}

export const STATUS_FILL: Record<string, string> = {
  success: '#e3f3ea',
  failure: '#fdecd9',
  running: '#dce9f8',
}

export const STATUS_STROKE: Record<string, string> = {
  success: '#27966b',
  failure: '#c0392b',
  running: '#3b6ea5',
}
