import { DwgType } from "albatros/enums";

// Простое module-level состояние для статус-бара. package.json теперь читает его
// напрямую через DIESEL: statusbar.polyline_info -> "label": "$(get_polyline_status)"
// (dynamics для этого не нужны — это механизм для выпадающих списков, не для текста).
const liveStatus = { text: "" };

// vec2 и vec3 — уже объявлены глобально в библиотеке (math_d.ts), свои не нужны
type Triangle = [vec2, vec2, vec2];

function unwrap(obj: any): any {
    return obj && typeof obj === "object" && "value" in obj ? obj.value : obj;
}

function toVec2(v: vec3): vec2 {
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
function signedArea(poly: vec2[]): number {
    let sum = 0;
    for (let i = 0; i < poly.length; i++) {
        sum += Math3d.vec2.cross(poly[i]!, poly[(i + 1) % poly.length]!);
    }
    return sum / 2;
}

// контур к обходу против часовой стрелки 
function ensureCCW(poly: vec2[]): vec2[] {
    return signedArea(poly) < 0 ? [...poly].reverse() : poly.slice();
}


// Пускаем луч вправо и считаем пересечения с рёбрами
// Если нечётное количество пересечений — точка внутри
function pointInPolygon(p: vec2, poly: vec2[]): boolean {
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

// Находим точку пересечения через готовую функцию библиотеки
function lineIntersection(p1: vec2, p2: vec2, p3: vec2, p4: vec2): vec2 {
    const l1 = Math3d.line2.makePoints({} as line2, p1, p2);
    const l2 = Math3d.line2.makePoints({} as line2, p3, p4);

    const target: vec2 = [0, 0];
    const ok = Math3d.line2.intersect(target, l1, l2);

    return ok ? target : p1; // прямые почти параллельны — на практике сюда не попадаем
}

// Пересечение отрезков: находим точку через Math3d.line2 (как в lineIntersection),
// а параметры t/u (позиция вдоль каждого отрезка) — через проекцию векторов (vec2.dot).
function segmentIntersection(
    p1: vec2, p2: vec2, p3: vec2, p4: vec2
): { point: vec2; t: number; u: number } | null {
    const l1 = Math3d.line2.makePoints({} as line2, p1, p2);
    const l2 = Math3d.line2.makePoints({} as line2, p3, p4);

    const point: vec2 = [0, 0];
    if (!Math3d.line2.intersect(point, l1, l2)) return null; // прямые параллельны

    const d12: vec2 = [0, 0];
    Math3d.vec2.sub(d12, p2, p1);
    const d34: vec2 = [0, 0];
    Math3d.vec2.sub(d34, p4, p3);

    const toPoint12: vec2 = [0, 0];
    Math3d.vec2.sub(toPoint12, point, p1);
    const toPoint34: vec2 = [0, 0];
    Math3d.vec2.sub(toPoint34, point, p3);

    const len12sqr = Math3d.vec2.lensqr(d12);
    const len34sqr = Math3d.vec2.lensqr(d34);
    if (len12sqr < 1e-18 || len34sqr < 1e-18) return null; // вырожденный отрезок

    const t = Math3d.vec2.dot(toPoint12, d12) / len12sqr;
    const u = Math3d.vec2.dot(toPoint34, d34) / len34sqr;

    if (t < -1e-9 || t > 1 + 1e-9 || u < -1e-9 || u > 1 + 1e-9) return null;

    return { point, t, u };
}
// Знак векторного произведения ребра a->b и вектора a->p:
// >=0 — точка слева от ребра ("внутри" для CCW-контура), <0 — справа ("снаружи")
function edgeSide(a: vec2, b: vec2, p: vec2): number {
    const edge: vec2 = [0, 0];
    Math3d.vec2.sub(edge, b, a);
    const toPoint: vec2 = [0, 0];
    Math3d.vec2.sub(toPoint, p, a);
    return Math3d.vec2.cross(edge, toPoint);
}

//отсечение
function clipAgainstOrientedConvex(subject: vec2[], orientedClip: vec2[]): vec2[] {
    let output: vec2[] = subject.slice();
    // Проходим по каждому ребру отсекающего полигона
    for (let i = 0; i < orientedClip.length && output.length > 0; i++) {
        const a = orientedClip[i]!;
        const b = orientedClip[(i + 1) % orientedClip.length]!;

        // точка "внутри", если она слева от направленного ребра a -> b
        const isInside = (p: vec2) => edgeSide(a, b, p) >= 0;

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
function intersectConvex(subject: vec2[], clip: vec2[]): vec2[] {
    return clipAgainstOrientedConvex(subject, ensureCCW(clip));
}

// Отсекаем subject по ОДНОМУ ребру a->b, оставляя часть СНАРУЖИ этого ребра
// (обычный Sutherland-Hodgman, только тест "внутри" развёрнут на противоположный)
function clipOutsideSingleEdge(subject: vec2[], a: vec2, b: vec2): vec2[] {
    const isOutside = (p: vec2) => edgeSide(a, b, p) < 0;

    const output: vec2[] = [];
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
function subtractConvex(subject: vec2[], clip: vec2[]): vec2[][] {
    const orientedClip = ensureCCW(clip);
    const pieces: vec2[][] = [];

    for (let i = 0; i < orientedClip.length; i++) {
        const a = orientedClip[i]!;
        const b = orientedClip[(i + 1) % orientedClip.length]!;
        const piece = clipOutsideSingleEdge(subject, a, b);
        if (piece.length >= 3) pieces.push(piece);
    }

    return pieces;
}

/* Триангуляция готового контура */
function triangulatePolygon(poly: vec2[]): Triangle[] {
    if (poly.length < 3) return [];

    const flat = new Float32Array(poly.length * 2);
    poly.forEach((p, i) => {
        flat[i * 2] = p[0];
        flat[i * 2 + 1] = p[1];
    });

    const indices = Math3d.geometry.triangulate(flat) as number[];
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
    point: vec2; // координаты точки
    isIntersection: boolean; // true = точка пересечения с другим полигоном
    isEntry?: boolean; // true = точка внутри
    linkId?: number; // ID для связывания пересечений
    visited?: boolean; // пометка для обхода
}

function buildUnionContours(subjectIn: vec2[], clipIn: vec2[]): vec2[][] {
    const subject = ensureCCW(subjectIn);
    const clip = ensureCCW(clipIn);

    interface EdgeCrossing {
        subjectEdge: number;  // индекс ребра
        subjectT: number; // параметр t 
        clipEdge: number; // индекс ребра
        clipU: number;  // параметр u
        point: vec2; 
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
        poly: vec2[],
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
    function markEntryExit(list: BoundaryNode[], otherPoly: vec2[]) {
        const n = list.length;
        for (let i = 0; i < n; i++) {
            const node = list[i]!;
            if (!node.isIntersection) continue;
            const next = list[(i + 1) % n]!.point;
            const midpoint: vec2 = [(node.point[0] + next[0]) / 2, (node.point[1] + next[1]) / 2];
            node.isEntry = pointInPolygon(midpoint, otherPoly);
        }
    }

    markEntryExit(subjectList, clip);

    const contours: vec2[][] = [];
    const step = 1;
    // Строим контуры объединения
    for (let startIdx = 0; startIdx < subjectList.length; startIdx++) {
        const startNode = subjectList[startIdx]!;
        if (!startNode.isIntersection || !startNode.isEntry || startNode.visited) continue;

        const contour: vec2[] = [];
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


enum BooleanMode {
    Intersection = "intersection",
    Union = "union",
    Difference = "difference"
}

const MODE_LABEL: Record<BooleanMode, string> = {
    [BooleanMode.Intersection]: "пересечения",
    [BooleanMode.Union]: "объединения",
    [BooleanMode.Difference]: "разности"
};

const MODE_COLOR: Record<BooleanMode, number> = {
    [BooleanMode.Intersection]: 3, // зелёный
    [BooleanMode.Union]: 4,        // голубой
    [BooleanMode.Difference]: 1    // красный
};

async function pickTwoPolylines(
    ctx: Context,
    cadview: any,
    label: string
): Promise<{ poly1: vec2[]; poly2: vec2[] } | null> {
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
    mode: BooleanMode.Intersection | BooleanMode.Difference,
    poly1: vec2[],
    poly2: vec2[]
): Promise<void> {
    const label = MODE_LABEL[mode];

    // Пересечение — всегда один контур (или пусто).
    // Разность — может состоять из нескольких кусков (по числу рёбер clip), поэтому
    // работаем с массивом контуров в обоих случаях.
    const resultPolys: vec2[][] =
        mode === BooleanMode.Intersection
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
    poly1: vec2[],
    poly2: vec2[]
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
            color: MODE_COLOR[BooleanMode.Union]
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

    if (mode === BooleanMode.Union) {
        await createUnionPolyline(ctx, editor, picked.poly1, picked.poly2);
    } else {
        await fillTrianglesForBooleanOp(ctx, editor, mode, picked.poly1, picked.poly2);
    }
}

// --- Общие хелперы: резинка, угол/длина, статус-бар ---

// Угол вектора start->cursor в радианах — нужен для editor.addArc и dc.arcto (там всё в радианах)
function angleRad(from: vec3, to: vec3): number {
    const dir: vec2 = [0, 0];
    Math3d.vec2.sub(dir, [to[0], to[1]], [from[0], from[1]]);
    return Math3d.vec2.angle(dir);
}

// Угол вектора start->cursor в градусах от горизонтали, 0..360
function angleFromHorizontal(start: vec3, cursor: vec3): number {
    let deg = (angleRad(start, cursor) * 180) / Math.PI;
    if (deg < 0) deg += 360;
    return deg;
}

function distance2D(start: vec3, cursor: vec3): number {
    return Math3d.vec2.distance([start[0], start[1]], [cursor[0], cursor[1]]);
}

// Обновляем статус-бар только когда текст реально изменился — иначе dispose()+create
// на каждый кадр (60/сек) не успевает отрисоваться.
function createStatusUpdater(ctx: Context) {
    let current: IDisposable | undefined;
    let lastText: string | undefined;
    return {
        update(text: string) {
            if (text === lastText) return; // ничего не изменилось — не дёргаем UI зря
            lastText = text;
            current?.dispose();
            current = ctx.setStatusBarMessage(text);
        },
        dispose() {
            current?.dispose();
            current = undefined;
            lastText = undefined;
        }
    };
}

// Резинка от start до курсора + дуга угла + запись длины/угла в статус-бар.
// getLast() — последнее значение для разового сообщения после клика.
function makeLengthAngleHandler(
    ctx: Context,
    start: vec3,
    statusUpdater?: ReturnType<typeof createStatusUpdater>
): { handler: DynamicCallback; getLast: () => { length: number; angleDeg: number } } {
    let last = { length: 0, angleDeg: 0 };

    const handler: DynamicCallback = (dc, _camera, cursor) => {
        const length = distance2D(start, cursor);
        const angleDegNormalized = angleFromHorizontal(start, cursor);
        const angleRadRaw = angleRad(start, cursor); // может быть отрицательным — для arcto это ок (направление дуги)
        last = { length, angleDeg: angleDegNormalized };

        const text = `Длина: ${length.toFixed(2)}   Угол: ${angleDegNormalized.toFixed(1)}°`;
        liveStatus.text = text;
        statusUpdater?.update(text);

        // резиновая нить до курсора
        dc.color = 0xff0000ff;
        dc.newpath([start[0], start[1]]);
        dc.lineto([cursor[0], cursor[1]]);
        dc.stroke(1);

        // опорная горизонталь для отсчёта угла
        const refLength = Math.max(length, 1e-6);
        dc.color = 0x80808080;
        dc.newpath([start[0], start[1]]);
        dc.lineto([start[0] + refLength, start[1]]);
        dc.stroke(1);

        // дуга угла между горизонталью и текущим направлением
        const arcRadius = Math.min(refLength * 0.3, 20) || 5;
        dc.color = 0x0000ffff;
        dc.arcto([start[0], start[1]], arcRadius, 0, angleRadRaw);
        dc.stroke(1);
    };

    return { handler, getLast: () => last };
}

// Рисует контур по отдельным отрезкам (не одним длинным путём) — так надёжнее рисуется.
function strokePolylinePreview(dc: DeviceContext, points: vec2[], closed: boolean, color: number): void {
    const edgeCount = closed ? points.length : points.length - 1;
    for (let i = 0; i < edgeCount; i++) {
        const a = points[i];
        const b = points[(i + 1) % points.length];
        if (!a || !b) continue;
        dc.color = color;
        dc.newpath(a);
        dc.lineto(b);
        dc.stroke(1);
    }
}

function getEditorAndCadview(ctx: Context): { cadview: any; editor: any } {
    const cadview = ctx.cadview;
    const drawing = ctx.app?.model as any;
    const editor = drawing?.layouts?.model?.editor?.();
    return { cadview, editor };
}

// Оборачивает изменения в транзакцию editor.beginEdit/endEdit — убирает повторение
// одинакового try/finally по всему файлу.
async function withEdit<T>(editor: any, fn: () => Promise<T>): Promise<T> {
    await editor.beginEdit();
    try {
        return await fn();
    } finally {
        await editor.endEdit();
    }
}

// invalidate()+repaint() всегда вызываются парой после изменений на чертеже
function refresh(cadview: any): void {
    cadview.invalidate();
    cadview.repaint();
}

/** Спросить номер цвета ACI (1-255) через простое окно ввода. undefined — если отменили. */
async function askAciColor(ctx: Context, current: number): Promise<number | undefined> {
    const input = await ctx.showInputBox({
        title: "Цвет линии",
        prompt: "Номер цвета ACI (1-255)",
        value: String(current),
        validateInput: (v) => {
            const n = parseInt(v, 10);
            return isNaN(n) || n < 1 || n > 255 ? "Введите число от 1 до 255" : undefined;
        }
    });
    if (input === undefined) return undefined;
    const parsed = parseInt(input, 10);
    return isNaN(parsed) ? undefined : parsed;
}

/** Вершины правильного N-угольника (вписанного или описанного) вокруг центра. */
function computeRegularPolygon(
    center: vec3,
    radius: number,
    sides: number,
    circumscribed: boolean
): vec2[] {
    const cx = center[0];
    const cy = center[1];

    // Для описанного многоугольника radius — расстояние до середины стороны,
    // а не до вершины, поэтому вершины чуть дальше (R = radius / cos(pi/sides)).
    const R = circumscribed ? radius / Math.cos(Math.PI / sides) : radius;

    const vertices: vec2[] = [];
    for (let i = 0; i < sides; i++) {
        const angle = (i / sides) * 2 * Math.PI - Math.PI / 2;
        vertices.push([cx + R * Math.cos(angle), cy + R * Math.sin(angle)]);
    }
    return vertices;
}

// Отрезок/луч/прямая — одна логика, рисуются цепочкой. Луч и прямая — это Line,
// далеко вытянутый в нужную сторону (отдельного типа сущности для них нет).
type LinearMode = "segment" | "ray" | "xline";

// Длина, на которую "тянем" луч/прямую — берём от размера текущего вида камеры
// (через ViewFrustum.box), чтобы линия гарантированно выходила за пределы экрана
// независимо от масштаба чертежа. Если по какой-то причине не получилось — запасное значение.
function computeFarDistance(cadview: any): number {
    try {
        const box: box3 = [0, 0, 0, 0, 0, 0];
        cadview.camera.frustum.box(box);
        const dx = (box as number[])[3]! - (box as number[])[0]!;
        const dy = (box as number[])[4]! - (box as number[])[1]!;
        const diag = Math.hypot(dx, dy);
        return diag > 0 ? diag * 2 : 1e5;
    } catch {
        return 1e5;
    }
}

function computeLinearEndpoints(mode: LinearMode, p1: vec3, p2: vec3, farDistance: number): { a: vec3; b: vec3 } {
    if (mode === "segment") return { a: p1, b: p2 };

    const dir: vec2 = [0, 0];
    Math3d.vec2.sub(dir, [p2[0], p2[1]], [p1[0], p1[1]]);
    if (Math3d.vec2.lensqr(dir) < 1e-18) return { a: p1, b: p2 }; // точки совпали — не тянем
    Math3d.vec2.normalize(dir, dir);

    if (mode === "ray") {
        const b: vec3 = [p1[0] + dir[0] * farDistance, p1[1] + dir[1] * farDistance, 0];
        return { a: p1, b };
    }

    // xline — тянем в обе стороны через p1
    const a: vec3 = [p1[0] - dir[0] * farDistance, p1[1] - dir[1] * farDistance, 0];
    const b: vec3 = [p1[0] + dir[0] * farDistance, p1[1] + dir[1] * farDistance, 0];
    return { a, b };
}

async function drawLinear(ctx: Context, mode: LinearMode): Promise<void> {
    const { cadview, editor } = getEditorAndCadview(ctx);
    if (!cadview || !editor) {
        ctx.showMessage("Нет доступа к редактору", "warning");
        return;
    }

    liveStatus.text = "";
    const statusUpdater = createStatusUpdater(ctx);

    const defaultColor = 7; // белый по умолчанию для всех линий (можно сменить через меню)
    let color = defaultColor;
    const farDistance = computeFarDistance(cadview); // для ray/xline — длина "вытягивания" линии

    const createdEntities: { entity: any; marker?: any }[] = [];
    const chainPoints: vec3[] = []; // p1 каждого созданного элемента, для "замкнуть"

    function buildAlts(opts: { canClose: boolean }): AlternativeCommands {
        const result: AlternativeCommands = {};
        if (opts.canClose) result.close = "Замкнуть (до первой точки)";
        if (createdEntities.length > 0) result.undo = "Отменить последний элемент";
        result.color = "Изменить цвет";
        result.exit = "Выход";
        return result;
    }

    async function askColor(): Promise<void> {
        const picked = await askAciColor(ctx, color);
        if (picked !== undefined) {
            color = picked;
            ctx.showMessage(`Цвет: ACI ${color}`);
        }
    }

    async function createEntity(a: vec3, b: vec3, originForMarker?: vec3): Promise<{ entity: any; marker?: any }> {
        const result = await withEdit(editor, async () => {
            const entity = await editor.addLine({ a, b, color });

            // Для луча дополнительно ставим маленький маркер в точке начала —
            // иначе на глаз луч и прямая неотличимы (у обоих просто длинная линия).
            let marker: any;
            if (entity && mode === "ray" && originForMarker) {
                marker = await editor.addCircle({
                    center: originForMarker,
                    radius: Math.max(farDistance * 0.005, 0.1),
                    color
                });
            }
            return { entity, marker };
        });
        refresh(cadview);
        return result;
    }

    async function eraseLast(): Promise<void> {
        const last = createdEntities.pop();
        chainPoints.pop();
        if (!last) {
            ctx.showMessage("Нечего отменять", "warning");
            return;
        }
        await withEdit(editor, async () => {
            await last.entity.erase();
            if (last.marker) await last.marker.erase();
        });
        refresh(cadview);
        ctx.showMessage("Последний элемент отменён");
    }

    let lastPoint: vec3 | undefined;

    try {
        while (true) {
            // Первая точка нового элемента — либо конец предыдущего (цепочка), либо запрос
            let p1: vec3;
            if (lastPoint) {
                p1 = lastPoint;
            } else {
                const r = await cadview.getpoint(
                    createdEntities.length === 0 ? "Укажите точку" : "Укажите след. точку",
                    buildAlts({ canClose: false })
                );

                if (typeof r === "string") {
                    if (r === "exit") break;
                    if (r === "undo") { await eraseLast(); continue; }
                    if (r === "color") { await askColor(); continue; }
                    // "close" здесь не имеет смысла — ещё нет ни одного элемента
                    continue;
                }
                p1 = r;
            }

            const { handler } = makeLengthAngleHandler(ctx, p1, statusUpdater);
            statusUpdater.update("Длина: 0.00   Угол: 0.0°"); // чтобы сразу были наши данные, а не значок по умолчанию
            const r2 = await cadview.getpoint("Укажите след. точку", buildAlts({ canClose: chainPoints.length > 0 }), undefined, handler);

            if (typeof r2 === "string") {
                if (r2 === "close" && chainPoints.length > 0) {
                    const first = chainPoints[0]!;
                    const { a, b } = computeLinearEndpoints(mode, p1, first, farDistance);
                    const created = await createEntity(a, b, p1);
                    if (created.entity) { createdEntities.push(created); chainPoints.push(p1); }
                    break;
                }
                if (r2 === "exit") break;
                if (r2 === "undo") { await eraseLast(); lastPoint = undefined; continue; }
                if (r2 === "color") { await askColor(); lastPoint = p1; continue; }
                lastPoint = undefined;
                continue;
            }

            const p2 = r2;
            const { a, b } = computeLinearEndpoints(mode, p1, p2, farDistance);
            const created = await createEntity(a, b, p1);

            if (!created.entity) {
                ctx.showMessage("Не удалось создать элемент", "error");
                break;
            }

            createdEntities.push(created);
            chainPoints.push(p1);
            lastPoint = mode === "segment" ? p2 : undefined; // для луча/прямой цепочка "точка-в-точку" не так естественна
        }
    } catch (e) {
        console.warn("Рисование прервано:", e);
    }

    statusUpdater.dispose();
    liveStatus.text = "";
    const noun = mode === "segment" ? "Отрезков" : mode === "ray" ? "Лучей" : "Прямых";
    ctx.showMessage(`${noun} создано: ${createdEntities.length}`);
}

export default {
    hello: (ctx: Context): void => {
        ctx.showMessage("Плагин загружен");
    },

    // Читает текущее "живое" значение длины/угла для статус-бара
    // (package.json: statusbar.polyline_info -> "label": "$(get_polyline_status)")
    get_polyline_status: (_ctx: Context): string => liveStatus.text,

    // Отрезок(ы) — можно рисовать цепочкой, ПКМ открывает меню
    drawSegment: (ctx: Context): Promise<void> => drawLinear(ctx, "segment"),

    // Луч — визуально реализован как Line, вытянутый далеко в одном направлении
    // (в этой версии API нет подтверждённого отдельного типа сущности "луч")
    drawRay: (ctx: Context): Promise<void> => drawLinear(ctx, "ray"),

    // Прямая — Line, вытянутый далеко в обе стороны через первую точку
    drawStraightLine: (ctx: Context): Promise<void> => drawLinear(ctx, "xline"),

    drawArc: async (ctx: Context): Promise<void> => {
        const { cadview, editor } = getEditorAndCadview(ctx);
        if (!cadview || !editor) {
            ctx.showMessage("Нет доступа к редактору", "warning");
            return;
        }

        const center = await cadview.getpoint("Центр дуги");
        if (typeof center === "string") {
            ctx.showMessage("Отменено", "warning");
            return;
        }

        liveStatus.text = "";
        const statusUpdater = createStatusUpdater(ctx);

        const startHandler = makeLengthAngleHandler(ctx, center, statusUpdater);
        statusUpdater.update("Длина: 0.00   Угол: 0.0°");
        const startPoint = await cadview.getpoint(
            "Радиус и начальный угол",
            { cancel: "Отмена" },
            undefined,
            startHandler.handler
        );
        if (typeof startPoint === "string") {
            statusUpdater.dispose();
            liveStatus.text = "";
            ctx.showMessage("Отменено", "warning");
            return;
        }

        const radius = distance2D(center, startPoint);
        const startAngleRad = angleRad(center, startPoint);
        ctx.showMessage(`Радиус: ${radius.toFixed(3)}`);

        // Динамический предпросмотр: сама дуга через dc.arcto, плюс линия к курсору,
        // плюс "живой" радиус/угол раствора дуги в статус-баре (change-gated).
        const previewArc: DynamicCallback = (dc, _camera, cursor) => {
            const cursorAngleRad = angleRad(center, cursor);
            let span = cursorAngleRad - startAngleRad;
            if (span < 0) span += Math.PI * 2;

            const text = `Радиус: ${radius.toFixed(2)}   Угол раствора: ${((span * 180) / Math.PI).toFixed(1)}°`;
            liveStatus.text = text;
            statusUpdater.update(text);

            dc.color = 0xff0000ff; // красная резинка — сама дуга
            dc.arcto([center[0], center[1]], radius, startAngleRad, span);
            dc.stroke(2);

            dc.color = 0x80808080; // вспомогательная линия к курсору
            dc.newpath([center[0], center[1]]);
            dc.lineto([cursor[0], cursor[1]]);
            dc.stroke(1);
        };

        statusUpdater.update(`Радиус: ${radius.toFixed(2)}   Угол раствора: 0.0°`);
        const endPoint = await cadview.getpoint(
            "Конечный угол",
            { cancel: "Отмена" },
            undefined,
            previewArc
        );
        if (typeof endPoint === "string") {
            statusUpdater.dispose();
            liveStatus.text = "";
            ctx.showMessage("Отменено", "warning");
            return;
        }

        const endAngleRad = angleRad(center, endPoint);
        let span = endAngleRad - startAngleRad;
        if (span < 0) span += Math.PI * 2;

        await withEdit(editor, () => editor.addArc({ center, radius, angle: startAngleRad, span, color: 7 }));
        refresh(cadview);

        statusUpdater.dispose();
        liveStatus.text = "";
        ctx.showMessage(`Дуга создана: радиус ${radius.toFixed(3)}, угол ${((span * 180) / Math.PI).toFixed(1)}°`);
    },

    // Правильный N-угольник: число сторон -> тип (вписанный/описанный) -> центр -> радиус
    drawN: async (ctx: Context): Promise<void> => {
        const { cadview, editor } = getEditorAndCadview(ctx);
        if (!cadview || !editor) {
            ctx.showMessage("Нет доступа к редактору", "warning");
            return;
        }

        liveStatus.text = "";

        const sidesInput = await ctx.showInputBox({
            title: "Количество сторон",
            prompt: "Введите число сторон (минимум 3)",
            value: "6",
            validateInput: (v) => {
                const n = parseInt(v, 10);
                return isNaN(n) || n < 3 ? "Введите число ≥ 3" : undefined;
            }
        });
        if (sidesInput === undefined) {
            ctx.showMessage("Отменено", "warning");
            return;
        }
        const sides = parseInt(sidesInput, 10);

        const typeChoice = await ctx.showQuickPick(
            ["Вписанный (вершины на окружности)", "Описанный (стороны касаются окружности)"],
            { title: "Выберите тип многоугольника" }
        );
        if (!typeChoice) {
            ctx.showMessage("Отменено", "warning");
            return;
        }
        const isCircumscribed = typeChoice.includes("Описанный");

        const center = await cadview.getpoint("Укажите центр многоугольника");
        if (typeof center === "string") {
            ctx.showMessage("Отменено", "warning");
            return;
        }

        const polygonStatus = createStatusUpdater(ctx);
        polygonStatus.update(`${sides}-угольник, ${isCircumscribed ? "Описанный" : "Вписанный"}, радиус: 0.000`);

        const previewPolygon: DynamicCallback = (dc, _camera, cursor) => {
            try {
                const radius = distance2D(center, cursor);
                const vertices = computeRegularPolygon(center, radius, sides, isCircumscribed);

                strokePolylinePreview(dc, vertices, true, 0xff0000ff); // красная резинка

                const typeText = isCircumscribed ? "Описанный" : "Вписанный";
                const text = `${sides}-угольник, ${typeText}, радиус: ${radius.toFixed(3)}`;
                liveStatus.text = text;
                polygonStatus.update(text);
            } catch (e) {
                // Если сюда попали — резинка ломается именно здесь. Смотрите текст ошибки в консоли.
                console.error("[drawN] Ошибка в предпросмотре многоугольника:", e);
            }
        };

        const radiusPoint = await cadview.getpoint(
            "Радиус многоугольника",
            { cancel: "Отмена" },
            undefined,
            previewPolygon
        );
        if (typeof radiusPoint === "string") {
            polygonStatus.dispose();
            liveStatus.text = "";
            ctx.showMessage("Отменено", "warning");
            return;
        }

        const radius = distance2D(center, radiusPoint);
        const vertices = computeRegularPolygon(center, radius, sides, isCircumscribed);

        if (vertices.length < 3) {
            polygonStatus.dispose();
            ctx.showMessage("Не удалось построить многоугольник", "error");
            return;
        }

        const vertices3d: vec3[] = vertices.map(([x, y]) => [x, y, 0]);

        await withEdit(editor, () =>
            editor.addPolyline({ vertices: vertices3d, flags: 1, color: 7, width: 0, elevation: 0 })
        );
        refresh(cadview);

        polygonStatus.dispose();
        liveStatus.text = "";
        const typeText = isCircumscribed ? "описанный" : "вписанный";
        ctx.showMessage(`${sides}-угольник (${typeText}) создан, ${vertices.length} вершин`);
    },

    // Прямоугольник по двум противоположным углам (оси совпадают с осями чертежа)
    drawRectangle: async (ctx: Context): Promise<void> => {
        const { cadview, editor } = getEditorAndCadview(ctx);
        if (!cadview || !editor) {
            ctx.showMessage("Нет доступа к редактору", "warning");
            return;
        }

        liveStatus.text = "";

        const corner1 = await cadview.getpoint("Первый угол прямоугольника");
        if (typeof corner1 === "string") {
            ctx.showMessage("Отменено", "warning");
            return;
        }

        const rectStatus = createStatusUpdater(ctx);
        rectStatus.update("Ширина: 0.000   Высота: 0.000");

        const previewRect: DynamicCallback = (dc, _camera, cursor) => {
            const corners: vec2[] = [
                [corner1[0], corner1[1]],
                [cursor[0], corner1[1]],
                [cursor[0], cursor[1]],
                [corner1[0], cursor[1]]
            ];
            strokePolylinePreview(dc, corners, true, 0xff0000ff); // красная резинка

            const w = Math.abs(cursor[0] - corner1[0]);
            const h = Math.abs(cursor[1] - corner1[1]);
            const text = `Ширина: ${w.toFixed(3)}   Высота: ${h.toFixed(3)}`;
            liveStatus.text = text;
            rectStatus.update(text);
        };

        const corner2 = await cadview.getpoint(
            "Противоположный угол",
            { cancel: "Отмена" },
            undefined,
            previewRect
        );
        if (typeof corner2 === "string") {
            rectStatus.dispose();
            liveStatus.text = "";
            ctx.showMessage("Отменено", "warning");
            return;
        }

        const vertices: vec3[] = [
            [corner1[0], corner1[1], 0],
            [corner2[0], corner1[1], 0],
            [corner2[0], corner2[1], 0],
            [corner1[0], corner2[1], 0]
        ];

        await withEdit(editor, () =>
            editor.addPolyline({ vertices, flags: 1, color: 7, width: 0, elevation: 0 })
        );
        refresh(cadview);

        rectStatus.dispose();
        liveStatus.text = "";
        const w = Math.abs(corner2[0] - corner1[0]);
        const h = Math.abs(corner2[1] - corner1[1]);
        ctx.showMessage(`Прямоугольник создан: ${w.toFixed(3)} x ${h.toFixed(3)}`);
    },

    // Полилиния с контекстным меню по ПКМ: отменить последнюю точку,
    // замкнуть, изменить ширину/цвет. Плюс индикация длины/угла текущего сегмента.
    drawUserLine: async (ctx: Context): Promise<void> => {
        const { cadview, editor } = getEditorAndCadview(ctx);
        if (!cadview || !editor) {
            ctx.showMessage("Нет доступа к редактору", "warning");
            return;
        }

        liveStatus.text = "";

        const vertices: vec3[] = [];
        let width = 0;
        let color = 7; // белый по умолчанию
        let closed = false;
        let exited = false;
        let previewEntity: any = undefined; // текущая "превью" полилиния, чтобы не плодить копии

        const alts: AlternativeCommands = {
            undo: "Отменить последнюю точку",
            close: "Замкнуть",
            width: "Изменить ширину",
            color: "Изменить цвет"
        };

        async function redrawPreview(): Promise<void> {
            await withEdit(editor, async () => {
                if (previewEntity) {
                    await previewEntity.erase();
                    previewEntity = undefined;
                }
                if (vertices.length >= 2) {
                    previewEntity = await editor.addPolyline({
                        vertices,
                        flags: closed ? 1 : 0,
                        color: 2,
                        width,
                        elevation: 0
                    });
                }
            });
            refresh(cadview);
        }

        async function askWidth(): Promise<void> {
            const input = await ctx.showInputBox({
                title: "Ширина полилинии",
                prompt: "Введите ширину полилинии",
                value: String(width),
                validateInput: (v) => (isNaN(parseFloat(v)) ? "Введите число" : undefined)
            });
            if (input === undefined) return;
            const parsed = parseFloat(input);
            if (!isNaN(parsed)) {
                width = Math.max(0, parsed);
                ctx.showMessage(`Ширина установлена: ${width}`);
            }
        }

        async function askColor(): Promise<void> {
            const picked = await askAciColor(ctx, color);
            if (picked !== undefined) {
                color = picked;
                ctx.showMessage(`Цвет: ACI ${color}`);
            }
        }

        const topStatus = createStatusUpdater(ctx);

        async function askExactLength(basePoint: vec3, currentAngleDeg: number): Promise<void> {
            const input = await ctx.showInputBox({
                title: "Точная длина сегмента",
                prompt: `Направление сейчас: ${currentAngleDeg.toFixed(1)}° (по нему и ляжет точка)`,
                validateInput: (v) => (isNaN(parseFloat(v)) ? "Введите число" : undefined)
            });
            if (input === undefined) return;
            const length = parseFloat(input);
            if (isNaN(length)) return;
            const rad = (currentAngleDeg * Math.PI) / 180;
            const newPoint: vec3 = [basePoint[0] + length * Math.cos(rad), basePoint[1] + length * Math.sin(rad), 0];
            vertices.push(newPoint);
            await redrawPreview();
        }

        async function askExactAngle(basePoint: vec3, currentLength: number): Promise<void> {
            const input = await ctx.showInputBox({
                title: "Точный угол сегмента",
                prompt: `Длина сейчас: ${currentLength.toFixed(3)} (с ней и ляжет точка)`,
                validateInput: (v) => (isNaN(parseFloat(v)) ? "Введите число" : undefined)
            });
            if (input === undefined) return;
            const angleDeg = parseFloat(input);
            if (isNaN(angleDeg)) return;
            const rad = (angleDeg * Math.PI) / 180;
            const newPoint: vec3 = [basePoint[0] + currentLength * Math.cos(rad), basePoint[1] + currentLength * Math.sin(rad), 0];
            vertices.push(newPoint);
            await redrawPreview();
        }

        try {
            while (!closed) {
                const basePoint = vertices.length > 0 ? vertices[vertices.length - 1] : undefined;
                const lengthAngle = basePoint ? makeLengthAngleHandler(ctx, basePoint, topStatus) : undefined;
                if (lengthAngle) topStatus.update("Длина: 0.00   Угол: 0.0°");

                const pointAlts: AlternativeCommands = basePoint
                    ? { ...alts, length: "Задать точную длину", angle: "Задать точный угол", exit: "Выход" }
                    : { ...alts, exit: "Выход" };

                const result = await cadview.getpoint(
                    vertices.length === 0 ? "Укажите точку" : "Укажите след. точку",
                    pointAlts,
                    undefined,
                    lengthAngle?.handler
                );

                if (result === "length" && basePoint && lengthAngle) {
                    await askExactLength(basePoint, lengthAngle.getLast().angleDeg);
                    continue;
                }

                if (result === "angle" && basePoint && lengthAngle) {
                    await askExactAngle(basePoint, lengthAngle.getLast().length);
                    continue;
                }

                if (result === "undo") {
                    vertices.pop();
                    await redrawPreview();
                    continue;
                }

                if (result === "close") {
                    if (vertices.length >= 3) closed = true;
                    else ctx.showMessage("Нужно минимум 3 точки, чтобы замкнуть", "warning");
                    continue;
                }

                if (result === "width") {
                    await askWidth();
                    await redrawPreview();
                    continue;
                }

                if (result === "color") {
                    await askColor();
                    continue;
                }

                if (result === "exit") {
                    exited = true;
                    break;
                }

                if (typeof result === "string" || !result) break; // Enter/Escape — обычное завершение

                vertices.push(result);
                await redrawPreview();
            }
        } catch (e) {
            console.warn("Рисование отменено:", e);
            topStatus.dispose();
            liveStatus.text = "";
            if (previewEntity) await previewEntity.erase();
            ctx.showMessage("Отменено", "warning");
            return;
        }

        topStatus.dispose();

        if (exited || vertices.length < 2) {
            if (previewEntity) await previewEntity.erase();
            ctx.showMessage("Отменено", "warning");
            return;
        }

        try {
            await withEdit(editor, async () => {
                if (previewEntity) await previewEntity.erase();
                await editor.addPolyline({ vertices, flags: closed ? 1 : 0, color, width, elevation: 0 });
            });
        } catch (e) {
            console.error("Не удалось создать полилинию:", e);
            ctx.showMessage("Ошибка при создании полилинии", "error");
            return;
        }
        refresh(cadview);

        liveStatus.text = "";
        ctx.showMessage(`Готово: ${vertices.length} точек${closed ? ", замкнута" : ""}`);
    },

    // Пересечение 
    showIntersection: (ctx: Context): Promise<void> => runBooleanOp(ctx, BooleanMode.Intersection),
    // Объединение 
    showUnion: (ctx: Context): Promise<void> => runBooleanOp(ctx, BooleanMode.Union),
    // Разность красным
    showDifference: (ctx: Context): Promise<void> => runBooleanOp(ctx, BooleanMode.Difference)
};