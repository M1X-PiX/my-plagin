import { DwgType } from "albatros/enums";
 
type vec3 = [number, number, number];
type Vec2 = [number, number];
type Triangle = [Vec2, Vec2, Vec2];
 
function unwrap(obj: any): any {
    return obj && typeof obj === "object" && "value" in obj ? obj.value : obj;
}
 
function toVec2(v: vec3): Vec2 {
    return [v[0], v[1]];
}
 
/** Вершины и признак замкнутости выбранной полилинии. */
function getPolylineData(raw: any): { vertices: vec3[]; closed: boolean } | null {
    const obj = unwrap(raw);
    let vertices: vec3[] | undefined = obj?.vertices ?? obj?.$data?.vertices;
    if (!vertices || vertices.length < 3) return null;
 
    const flags: number = obj?.flags ?? obj?.$data?.flags ?? 0;
    const closed = (flags & 0x1) !== 0;
 
    // проверка на дубль последней вершины
    const first = vertices[0]!;
    const last = vertices[vertices.length - 1]!;
    const eps = 1e-9;
    if (Math.abs(first[0] - last[0]) < eps && Math.abs(first[1] - last[1]) < eps) {
        vertices = vertices.slice(0, -1);
    }
 
    return vertices.length >= 3 ? { vertices, closed } : null;
}
//если положительно, то обход против часовой
function signedArea(poly: Vec2[]): number {
    let sum = 0;
    for (let i = 0; i < poly.length; i++) {
        const [x1, y1] = poly[i]!;
        const [x2, y2] = poly[(i + 1) % poly.length]!;
        sum += x1 * y2 - x2 * y1;
    }
    return sum / 2;
}
 
// контур к обходу против часовой стрелки 
function ensureCCW(poly: Vec2[]): Vec2[] {
    return signedArea(poly) < 0 ? [...poly].reverse() : poly.slice();
}
 
 
// Пускаем луч вправо и считаем пересечения с рёбрами
// Если нечётное количество пересечений — точка внутри
function pointInPolygon(p: Vec2, poly: Vec2[]): boolean {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
        const [xi, yi] = poly[i]!;
        const [xj, yj] = poly[j]!;
        const crosses =
            yi > p[1] !== yj > p[1] &&
            p[0] < ((xj - xi) * (p[1] - yi)) / (yj - yi) + xi;
        if (crosses) inside = !inside;
    }
    return inside;
}
 
// Находим точку пересечения
function lineIntersection(p1: Vec2, p2: Vec2, p3: Vec2, p4: Vec2): Vec2 {
    const [x1, y1] = p1, [x2, y2] = p2, [x3, y3] = p3, [x4, y4] = p4;
    const d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (Math.abs(d) < 1e-12) return p1; // прямые почти параллельны
    const t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / d;
    return [x1 + t * (x2 - x1), y1 + t * (y2 - y1)];
}
 
// Пересечение отрезков
function segmentIntersection(
    p1: Vec2, p2: Vec2, p3: Vec2, p4: Vec2
): { point: Vec2; t: number; u: number } | null {
    const [x1, y1] = p1, [x2, y2] = p2, [x3, y3] = p3, [x4, y4] = p4;
    const d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (Math.abs(d) < 1e-9) return null;
 
    const t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / d;
    const u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / d;
    if (t < -1e-9 || t > 1 + 1e-9 || u < -1e-9 || u > 1 + 1e-9) return null;
 
    return { point: [x1 + t * (x2 - x1), y1 + t * (y2 - y1)], t, u };
}
//отсечение
function clipAgainstOrientedConvex(subject: Vec2[], orientedClip: Vec2[]): Vec2[] {
    let output: Vec2[] = subject.slice();
    // Проходим по каждому ребру отсекающего полигона
    for (let i = 0; i < orientedClip.length && output.length > 0; i++) {
        const a = orientedClip[i]!;
        const b = orientedClip[(i + 1) % orientedClip.length]!;
 
        // точка "внутри", если она слева от направленного ребра a -> b
        const isInside = (p: Vec2) => (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0;
 
        const input = output;
        output = [];
         // Проходим по всем рёбрам
        for (let j = 0; j < input.length; j++) {
            const current = input[j]!;
            const prev = input[(j - 1 + input.length) % input.length]!;
            const currentInside = isInside(current);
            const prevInside = isInside(prev);
 
            if (currentInside) {
                if (!prevInside) output.push(lineIntersection(prev, current, a, b));
                output.push(current);
            } else if (prevInside) {
                output.push(lineIntersection(prev, current, a, b));
            }
        }
    }
 
    return output;
}
 
/* Пересечение subject, обрезанный по  clip. */
function intersectConvex(subject: Vec2[], clip: Vec2[]): Vec2[] {
    return clipAgainstOrientedConvex(subject, ensureCCW(clip));
}
 
// Отсекаем subject по ОДНОМУ ребру a->b, оставляя часть СНАРУЖИ этого ребра
// (обычный Sutherland-Hodgman, только тест "внутри" развёрнут на противоположный)
function clipOutsideSingleEdge(subject: Vec2[], a: Vec2, b: Vec2): Vec2[] {
    const isOutside = (p: Vec2) => (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) < 0;
 
    const output: Vec2[] = [];
    for (let j = 0; j < subject.length; j++) {
        const current = subject[j]!;
        const prev = subject[(j - 1 + subject.length) % subject.length]!;
        const currentOutside = isOutside(current);
        const prevOutside = isOutside(prev);
 
        if (currentOutside) {
            if (!prevOutside) output.push(lineIntersection(prev, current, a, b));
            output.push(current);
        } else if (prevOutside) {
            output.push(lineIntersection(prev, current, a, b));
        }
    }
    return output;
}
 
/*
 * Разность subject \ clip (clip выпуклый).
 * По правилу де Моргана "снаружи выпуклого clip" — это ОБЪЕДИНЕНИЕ областей
 * "снаружи каждого отдельного ребра clip", а не пересечение (одно общее AND-отсечение
 * по всем рёбрам сразу почти всегда даёт пустоту — снаружи одного ребра обычно
 * означает "внутри" противоположного, и такая точка не проходит все проверки разом).
 * Поэтому отсекаем subject по каждому ребру clip отдельно и собираем все куски.
 */
function subtractConvex(subject: Vec2[], clip: Vec2[]): Vec2[][] {
    const orientedClip = ensureCCW(clip);
    const pieces: Vec2[][] = [];
 
    for (let i = 0; i < orientedClip.length; i++) {
        const a = orientedClip[i]!;
        const b = orientedClip[(i + 1) % orientedClip.length]!;
        const piece = clipOutsideSingleEdge(subject, a, b);
        if (piece.length >= 3) pieces.push(piece);
    }
 
    return pieces;
}
 
/* Триангуляция готового контура */
function triangulatePolygon(poly: Vec2[]): Triangle[] {
    if (poly.length < 3) return [];
 
    const flat = new Float32Array(poly.length * 2);
    poly.forEach((p, i) => {
        flat[i * 2] = p[0];
        flat[i * 2 + 1] = p[1];
    });
 
    const indices = (globalThis as any).Math3d.geometry.triangulate(flat) as number[];
    const triangles: Triangle[] = [];
 
    for (let i = 0; i < indices.length; i += 3) {
        const ia = indices[i], ib = indices[i + 1], ic = indices[i + 2];
        if (ia === undefined || ib === undefined || ic === undefined) continue;
 
        const a = poly[ia], b = poly[ib], c = poly[ic];
        if (!a || !b || !c) continue;
 
        triangles.push([a, b, c]);
    }
 
    return triangles;
}
 
interface BoundaryNode {
    point: Vec2; // координаты точки
    isIntersection: boolean; // true = точка пересечения с другим полигоном
    isEntry?: boolean; // true = точка внутри
    linkId?: number; // ID для связывания пересечений
    visited?: boolean; // пометка для обхода
}
 
function buildUnionContours(subjectIn: Vec2[], clipIn: Vec2[]): Vec2[][] {
    const subject = ensureCCW(subjectIn);
    const clip = ensureCCW(clipIn);
 
    interface EdgeCrossing {
        subjectEdge: number;  // индекс ребра
        subjectT: number; // параметр t 
        clipEdge: number; // индекс ребра
        clipU: number;  // параметр u
        point: Vec2; 
        id: number;
    }
 
    const crossings: EdgeCrossing[] = [];
    let nextId = 0;
    // проходим порёбрам обоих полигонов и ищем пересечения
    for (let i = 0; i < subject.length; i++) {
        const a1 = subject[i]!, a2 = subject[(i + 1) % subject.length]!;
        for (let j = 0; j < clip.length; j++) {
            const b1 = clip[j]!, b2 = clip[(j + 1) % clip.length]!;
            const hit = segmentIntersection(a1, a2, b1, b2);
            if (hit) {
                crossings.push({
                    subjectEdge: i, subjectT: hit.t,
                    clipEdge: j, clipU: hit.u,
                    point: hit.point, id: nextId++
                });
            }
        }
    }
 
    // контуры не пересекаются 
    if (crossings.length === 0) {
        if (pointInPolygon(subject[0]!, clip)) return [clip];
        if (pointInPolygon(clip[0]!, subject)) return [subject];
        return [subject, clip];
    }
    //вставляем точки пересечения между вершинами
    function buildBoundaryList(
        poly: Vec2[],
        edgeKey: "subjectEdge" | "clipEdge",
        paramKey: "subjectT" | "clipU"
    ): BoundaryNode[] {
        const list: BoundaryNode[] = [];
        for (let i = 0; i < poly.length; i++) {
            list.push({ point: poly[i]!, isIntersection: false });
 
            const onThisEdge = crossings
                .filter((c) => (c as any)[edgeKey] === i)
                .sort((a, b) => (a as any)[paramKey] - (b as any)[paramKey]);
 
            for (const c of onThisEdge) {
                list.push({ point: c.point, isIntersection: true, linkId: c.id });
            }
        }
        return list;
    }
 
    const subjectList = buildBoundaryList(subject, "subjectEdge", "subjectT");
    const clipList = buildBoundaryList(clip, "clipEdge", "clipU");
 
    const subjectIndexByLink = new Map<number, number>();
    subjectList.forEach((n, idx) => {
        if (n.isIntersection && n.linkId !== undefined) subjectIndexByLink.set(n.linkId, idx);
    });
    const clipIndexByLink = new Map<number, number>();
    clipList.forEach((n, idx) => {
        if (n.isIntersection && n.linkId !== undefined) clipIndexByLink.set(n.linkId, idx);
    });
 
    // Помечаем каждую точку пересечения входим в другой контур или выходим
    function markEntryExit(list: BoundaryNode[], otherPoly: Vec2[]) {
        const n = list.length;
        for (let i = 0; i < n; i++) {
            const node = list[i]!;
            if (!node.isIntersection) continue;
            const next = list[(i + 1) % n]!.point;
            const midpoint: Vec2 = [(node.point[0] + next[0]) / 2, (node.point[1] + next[1]) / 2];
            node.isEntry = pointInPolygon(midpoint, otherPoly);
        }
    }
 
    markEntryExit(subjectList, clip);
 
    const contours: Vec2[][] = [];
    const step = 1;
    // Строим контуры объединения
    for (let startIdx = 0; startIdx < subjectList.length; startIdx++) {
        const startNode = subjectList[startIdx]!;
        if (!startNode.isIntersection || !startNode.isEntry || startNode.visited) continue;
 
        const contour: Vec2[] = [];
        let onSubject = true;
        let currentList: BoundaryNode[] = subjectList;
        let idx = startIdx;
        let closed = false;
 
        const maxSteps = (subjectList.length + clipList.length) * 2 + 10;
        for (let guard = 0; guard < maxSteps; guard++) {
            const node = currentList[idx]!;
 
            // Вернулись  откуда начали контур замкнут
            if (onSubject && node.isIntersection && node.linkId === startNode.linkId && contour.length > 0) {
                closed = true;
                break;
            }
 
            if (node.isIntersection && onSubject) node.visited = true;
            contour.push(node.point);
 
            if (node.isIntersection) {
                if (onSubject) {
                    const target = clipIndexByLink.get(node.linkId!);
                    if (target === undefined) break;
                    currentList = clipList;
                    idx = target;
                    onSubject = false;
                } else {
                    const target = subjectIndexByLink.get(node.linkId!);
                    if (target === undefined) break;
                    currentList = subjectList;
                    idx = target;
                    onSubject = true;
                }
            }
 
            idx = (idx + step + currentList.length) % currentList.length;
        }
 
        if (closed && contour.length >= 3) contours.push(contour);
    }
 
    return contours;
}
 
 
type BooleanMode = "intersection" | "union" | "difference";
 
const MODE_LABEL: Record<BooleanMode, string> = {
    intersection: "пересечения",
    union: "объединения",
    difference: "разности"
};
 
const MODE_COLOR: Record<BooleanMode, number> = {
    intersection: 3, // зелёный
    union: 4,        // голубой
    difference: 1    // красный
};
 
async function pickTwoPolylines(
    ctx: Context,
    cadview: any,
    label: string
): Promise<{ poly1: Vec2[]; poly2: Vec2[] } | null> {
    const isPolyline: ObjectPredicate = (obj) => unwrap(obj)?.type === DwgType.polyline;
 
    let first: any, second: any;
    try {
        first = await cadview.getobject(`Выберите первую полилинию для ${label} (замкнутую)`, undefined, isPolyline);
        if (typeof first === "string") {
            ctx.showMessage("Отменено", "warning");
            return null;
        }
 
        second = await cadview.getobject(`Выберите вторую полилинию для ${label} (замкнутую)`, undefined, isPolyline);
        if (typeof second === "string") {
            ctx.showMessage("Отменено", "warning");
            return null;
        }
    } catch (e) {
        console.warn("Выбор отменён:", e);
        ctx.showMessage("Выбор отменён", "warning");
        return null;
    }
 
    const data1 = getPolylineData(first);
    const data2 = getPolylineData(second);
 
    if (!data1 || !data2) {
        ctx.showMessage("Не удалось прочитать вершины выбранных полилиний", "error");
        return null;
    }
 
    return { poly1: data1.vertices.map(toVec2), poly2: data2.vertices.map(toVec2) };
}
 
async function fillTrianglesForBooleanOp(
    ctx: Context,
    editor: any,
    mode: "intersection" | "difference",
    poly1: Vec2[],
    poly2: Vec2[]
): Promise<void> {
    const label = MODE_LABEL[mode];
 
    // Пересечение — всегда один контур (или пусто).
    // Разность — может состоять из нескольких кусков (по числу рёбер clip), поэтому
    // работаем с массивом контуров в обоих случаях.
    const resultPolys: Vec2[][] =
        mode === "intersection"
            ? (() => {
                  const poly = intersectConvex(poly1, poly2);
                  return poly.length >= 3 ? [poly] : [];
              })()
            : subtractConvex(poly1, poly2);
 
    if (resultPolys.length === 0) {
        ctx.showMessage(`Область ${label} не найдена`, "warning");
        return;
    }
 
    const triangles = resultPolys.flatMap(triangulatePolygon);
    if (triangles.length === 0) {
        ctx.showMessage(`Область ${label} не найдена`, "warning");
        return;
    }
 
    await editor.beginEdit();
    try {
        for (const [p1, p2, p3] of triangles) {
            await editor.addSolid({
                a: [p1[0], p1[1], 0],
                b: [p2[0], p2[1], 0],
                c: [p3[0], p3[1], 0],
                color: MODE_COLOR[mode]
            });
        }
    } finally {
        await editor.endEdit();
    }
 
    ctx.showMessage(`Область ${label} закрашена: ${triangles.length} треугольников`);
}
 
async function createUnionPolyline(
    ctx: Context,
    editor: any,
    poly1: Vec2[],
    poly2: Vec2[]
): Promise<void> {
    const contours = buildUnionContours(poly1, poly2);
    const contour = contours.reduce(
        (best, c) => (c.length > best.length ? c : best),
        contours[0] ?? []
    );
 
    if (contour.length < 3) {
        ctx.showMessage("Не удалось построить контур объединения", "error");
        return;
    }
 
    // Рисуем контур 
    const vertices: vec3[] = contour.map(([x, y]) => [x, y, 0]);
 
    await editor.beginEdit();
    try {
        await editor.addPolyline({
            vertices,
            flags: 1, // замкнутая
            color: MODE_COLOR.union
        });
    } finally {
        await editor.endEdit();
    }
 
    ctx.showMessage(`Полилиния объединения создана (${vertices.length} вершин)`);
}
 
async function runBooleanOp(ctx: Context, mode: BooleanMode): Promise<void> {
    const cadview = ctx.cadview;
    const drawing = ctx.app?.model as any;
    const editor = drawing?.layouts?.model?.editor?.();
 
    if (!cadview || !editor) {
        ctx.showMessage("Нет доступа к редактору", "warning");
        return;
    }
 
    const picked = await pickTwoPolylines(ctx, cadview, MODE_LABEL[mode]);
    if (!picked) return;
 
    if (mode === "union") {
        await createUnionPolyline(ctx, editor, picked.poly1, picked.poly2);
    } else {
        await fillTrianglesForBooleanOp(ctx, editor, mode, picked.poly1, picked.poly2);
    }
}
 
export default {
    hello: (ctx: Context): void => {
        ctx.showMessage("Плагин загружен");
    },
 
    drawUserLine: async (ctx: Context): Promise<void> => {
        const cadview = ctx.cadview;
        const drawing = ctx.app?.model as any;
        const editor = drawing?.layouts?.model?.editor?.();
 
        if (!cadview || !editor) {
            ctx.showMessage("Нет доступа к редактору", "warning");
            return;
        }
 
        const vertices: vec3[] = [];
        ctx.setStatusBarMessage("Начните рисовать полилинию");
 
        while (true) {
            const result = await cadview.getpoint(
                vertices.length === 0 ? "Первая точка" : "Следующая точка (Enter — завершить)"
            );
 
            if (typeof result === "string") break;
 
            const p: vec3 = [result?.[0] ?? 0, result?.[1] ?? 0, 0];
            vertices.push(p);
 
            if (vertices.length >= 2) {
                await editor.beginEdit();
                try {
                    await editor.addPolyline({
                        vertices,
                        flags: vertices.length > 2 ? 1 : 0,
                        color: 2,
                        width: 0,
                        elevation: 0
                    });
                } finally {
                    await editor.endEdit();
                }
            }
 
            ctx.setStatusBarMessage(`Точек: ${vertices.length}`);
        }
 
        if (vertices.length >= 2) {
            await editor.beginEdit();
            try {
                await editor.addPolyline({ vertices, flags: 0, color: 1 });
            } finally {
                await editor.endEdit();
            }
        }
 
        ctx.showMessage(`Готово: ${vertices.length} точек`);
    },
 
    // Пересечение 
    showIntersection: (ctx: Context): Promise<void> => runBooleanOp(ctx, "intersection"),
    // Объединение 
    showUnion: (ctx: Context): Promise<void> => runBooleanOp(ctx, "union"),
    // Разность красным
    showDifference: (ctx: Context): Promise<void> => runBooleanOp(ctx, "difference")
};
 