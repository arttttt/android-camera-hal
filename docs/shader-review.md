# ISP Shader Review — Performance Focus

Ревью шейдеров в `isp/vulkan/shaders/` с прицелом на Kepler SM 3.2 (GK20A,
Tegra K1 / Xiaomi Mi Pad 1 "mocha").

Шейдеры под ревью:

- `Blit.h` — full-screen triangle + fragment для scratch → gralloc blit
- `DemosaicCompute.h` — Malvar-He-Cutler 5×5 + WB/CCM/gamma
- `RgbaToNv12.h` — RGBA8 → NV12 single-dispatch
- `StatsCompute.h` — 16×16 patch histogram + Tenengrad sharpness + RGB mean

---

## Сводка приоритетов

| # | Файл | Проблема | Приоритет | Ожидаемый выигрыш на 1080p |
|---|------|----------|-----------|----------------------------|
| 1 | `DemosaicCompute.h` | Целочисленное деление в `readPixel`, 13 раз на пиксель | **P1** | ~3–5 мс/кадр |
| 2 | `DemosaicCompute.h` | Нет shared-тайла для Bayer — 13 некогерентных SSBO-чтений на выход | **P1** | ~5–10 мс/кадр (вместе перекрывает #1) |
| 3 | `DemosaicCompute.h` | `pow()` в sRGB-гамме 3× на пиксель при `doIsp=1` | **P2** | ~1–3 мс/кадр |
| 4 | `DemosaicCompute.h` | WG 8×8 ограничивает occupancy; 16×16 даёт +warps | **P2** | сам по себе маленький, синергичен с #2 |
| 5 | `DemosaicCompute.h` | `doIsp` как uniform-if — specialization constant сэкономит dead-code | **P3** | минимальный |
| 6 | `Blit.h` | Скейл-ветвь делает 2 vec2-деления на фрагмент | **P3** | < 0.5 мс |
| 7 | `RgbaToNv12.h` | `stride = pc.w / 4` на каждую инвокацию | **P3** | < 0.2 мс |
| 8 | `StatsCompute.h` | В целом оптимизирован; замечаний по перфу нет | — | — |

---

## 1. `DemosaicCompute.h` — самая богатая на win-ы зона

### P1. Целочисленное деление в горячем пути (`readPixel`)

```glsl
uint sub   = raw > params.blackLevel ? raw - params.blackLevel : 0u;
uint denom = maxv - params.blackLevel;
return (sub * 255u) / denom;
```

На Kepler целочисленное деление эмулируется, ~20–30 тактов. `readPixel`
вызывается **13 раз на выходной пиксель** (через макрос `PX`). При 1080p это
≈26M делений — заметная часть бюджета шейдера.

`denom` — униформа по всему диспатчу (`blackLevel` из SSBO-параметров,
`maxv` зависит только от `is16bit`). Хост может прямо посчитать
`float denomRecip255 = 255.0f / (maxv - blackLevel)` и положить в `IspParams`.
Тогда тело упрощается:

```glsl
float readPixelF(uint x, uint y) {
    // ...выборка raw как и сейчас...
    return max(float(raw) - float(params.blackLevel), 0.0)
           * params.denomRecip255;
}
```

`PX()` сразу возвращает `float`, лишний `float(readPixel())`-каст пропадает.

**Изменение в `isp/IspParams.h`:** одно новое поле `float denomRecip255` в
конце структуры (и апдейт `std430`-макета SSBO), хост заполняет его исходя из
`is16bit` + `blackLevel`.

### P1. Нет shared-тайла для Bayer — 13 чтений из SSBO на выходной пиксель

Текущий шейдер для каждого выходного пикселя делает 13 `inBuf.data[idx>>N]`.
Для `PX(-2, ±2)` рассинхрон по строкам — два ряда выше/ниже центра, L2 при
таком шаблоне работает плохо (особенно под нагрузкой от соседних
SM-резидентных WG).

Классическое решение — cooperative tile-load в shared:

```glsl
// WG 8x8, halo 2 по каждой стороне → тайл 12x12 = 144 пикселя
shared float tile[12][12];
```

64 потока кладут ~2.25 значения каждый (цикл по `tid`), `barrier()`, дальше
`PX(dx, dy)` читает из `tile[local_y+2+dy][local_x+2+dx]`. Преобразование через
`readPixelF` делается **один раз на пиксель** вместо 13. Экономит и деления
(P1 выше уходит автоматически), и глобальные чтения.

**Прикидка:**

- 13 → ~2.25 SSBO-чтений на пиксель
- ~200 тактов/чтение при частичных L2-промахах
- Экономия ~2000 тактов/пиксель
- На 2M пикселей, 192 CUDA-cores @ ~950 МГц → **до ~20 мс/кадр**

В связке с bump до WG 16×16 (tile 20×20 = 400 пикселей, ~1.56 чтения/поток)
смягчается register-pressure при более крупных WG и повышается occupancy.

### P2. `pow()` для sRGB-гаммы

```glsl
float srgbGamma(float lin) {
    return (lin <= 0.0031308)
        ? lin * 12.92
        : 1.055 * pow(lin, 1.0/2.4) - 0.055;
}
```

Вызывается 3 раза на пиксель при `doIsp=1`. `pow()` на Kepler =
`exp2(y * log2(x))` через SFU, ~8–12 тактов на вызов. Плюс input — это `int`
из `[0,255]`, т.е. разумных значений всего 256 штук.

Варианты по убыванию "прямолинейности":

1. **256-элементный LUT как UBO** — хост предвычисляет
   `uint8_t sGammaLut[256]`, шейдер читает `gammaLut[rr]`. L1 const-cache
   моментально прогревается. 1–2 такта на выборку, экономит ~30 тактов на
   пиксель × 3 канала.
2. **sRGB формат в ImageView** — если `mScratchImg` создать как
   `R8G8B8A8_UNORM`, а для blit-path сделать `ImageView` с
   `R8G8B8A8_SRGB`, ROP в `recordGrallocBlit` сам сделает линейное→sRGB.
   Убирает `srgbGamma` из демозаика целиком. Потребуется перекомпоновать
   порядок CCM/WB, чтобы результат оставался в линейном пространстве до
   blit'а. Архитектурно чище, но больше работы.
3. **Полиномиальная аппроксимация** (Хорнеровская форма 3–4 порядка).
   Дешевле `pow`, дороже LUT.

При выполнении P1+P2 (tile + reciprocal) LUT закроет этот пункт малыми
силами.

### P2. WG 8×8 занижает occupancy на Kepler

- Kepler SMX: max 2048 резидентных thread, max 16 blocks, 64 KB register file.
- При ~30–40 регистрах на поток 8×8 = 64 threads/WG → 16-WG лимит по блокам
  → 16×64 = 1024 threads резидентных ≈ 50 % occupancy.
- 16×16 = 256 threads/WG + shared tile ~1.6 KB → ~6 WG, 1536 threads
  резидентных ≈ 75 % occupancy.

16×16 также амортизирует cooperative load (больше потоков на тот же
маленький halo).

### P3. `doIsp` как uniform-условие

```glsl
if (params.doIsp != 0u) { ... WB, CCM, gamma ... }
```

Условие одинаково на весь диспатч → warp-divergence нет. Но компилятор держит
обе ветви в скомпилированном шейдере. Specialization constant уронит мёртвую
ветвь:

```glsl
layout(constant_id = 0) const uint kDoIsp = 1u;
// ...
if (kDoIsp != 0u) { ... }
```

Хост создаёт две pipeline-вариации (`doIsp=0` / `doIsp=1`), переключается в
момент, когда `doIsp` меняется. Маленький выигрыш (меньше uniform-загрузок),
заметен только если ISP-путь временами выключается.

### Корректность — всё чисто

- Маска фазы Bayer (`rX = phase & 1; rY = (phase>>1) & 1`) — совпадает с
  кодировкой V4L2 fourcc, без off-by-one.
- Диапазоны промежуточных `int`: `ccm[i]*R` при `ccm ∈ Q10 (~±2048)` и
  `R ∈ [0,255]` — максимум ~1.5M на сумме трёх, влезает в int32. OK.
- Clamp'ы на границах (`ix`, `iy` через `clamp`) — корректно.
- `imageStore(vec4(R,G,B,255.0) / 255.0)` — driver обычно хоистит `1/255.0`,
  но явное `* (1.0/255.0)` гарантирует это.

---

## 2. `Blit.h` — почти оптимален

### P3. Скейл-ветвь: 2 uniform-деления на фрагмент

```glsl
vec2 uv = (vec2(pc.cropX, pc.cropY)
         + gl_FragCoord.xy * vec2(pc.cropW, pc.cropH)
                           / vec2(pc.outW, pc.outH))
         / vec2(pc.srcW, pc.srcH);
```

`cropW/outW` и `1/srcW` — константы на draw-call, не per-fragment. Большинство
driver'ов хоистит scalar-math в preamble, но на старом Tegra-компиляторе не
гарантировано. Предвычислить на хосте и передать через push:

```glsl
layout(push_constant) uniform BlitPC {
    vec2 cropOffset;   // (cropX, cropY)
    vec2 scaleXY;      // (cropW/outW/srcW, cropH/outH/srcH) — объединено
    // identity-path всё ещё нужны cropX, cropY, cropW, cropH...
} pc;
```

Или хотя бы `vec2 invSrc` + `vec2 cropScale` отдельно. Фрагментов ~2M на
1080p, эффект < 0.5 мс — lazy-fix.

### Корректность — OK

- Full-screen triangle math верный: `vkCmdDraw(cb, 3, ...)` + треугольник
  (-1,-1), (3,-1), (-1,3). Проверено в `recordGrallocBlit`.
- Identity-branch: `ivec2(gl_FragCoord.xy) = ivec2(x, y)` после truncation из
  `(x+0.5, y+0.5)` (Vulkan default) — корректно.
- Identity не использует сэмплер — `texelFetch` даёт 1 тексель, filter
  пропускается. Правильно.
- Скейл-ветвь использует `texture()` с `LINEAR + CLAMP_TO_EDGE` —
  HW-bilinear, cross-res-scale honorно.
- Uniform-branch `if (cropW==outW && cropH==outH)` — дивергенции нет (push
  одинаков для всего draw'а).

---

## 3. `RgbaToNv12.h` — аккуратный, одна мелочь

### P3. `stride = pc.w / 4` в теле

Каждая инвокация делает `int stride = pc.w / 4` и использует его 3 раза.
Один целочисленный divide per invocation — мелочь, но посчитать на хосте
тривиально:

```cpp
pc.stride = (int32_t)(width / 4);  // уже есть uvBase = w*h/4
```

### Memory traffic — всё хорошо

Каждый поток читает 8 пикселей и пишет 3 uint'а. Соседние по warp потоки
работают с адресами, сдвинутыми на 4 пикселя горизонтально → **пересечений
выборок по warp нет**, L1 tex-cache не помогает, но и не должен — scratch
читается ровно один раз на пиксель. Оптимально для RGBA→NV12.

### Корректность — OK

- Хрома 2×2 downsample средним, chroma site "center" — стандартная практика
  для NV12.
- BT.601 limited-range — согласовано с тем, что ожидает Android по умолчанию
  без colour-space метаданных.
- Все записи aligned по uint, без atomics.
- Требование `width%4==0 && height%2==0` утверждено assert'ами в
  `ensureBuffers`.

### Возможное, но не обязательное

- Можно было бы **слить stats-compute и RGBA→NV12 в один пасс** (оба читают
  scratch ровно один раз), но у них разная геометрия диспатча (16×16 WG по
  патчам vs 4×2-блоки на invocation) — слияние усложнит код, выигрыш
  сомнительный.

---

## 4. `StatsCompute.h` — хорошо продуман, мелкие замечания

Этот самый свежий и самый аккуратный из четырёх. Решения обоснованы в
заголовке и согласуются с реальностью GK20A:

- **Per-warp privatized histogram** (`sHist[8][128]`) — правильная стратегия
  для Kepler: нулевая cross-warp contention, intra-warp capнут 32-way.
- **Без shared luma tile** — на патче 120×67 (1080p/16) full-tile в shared
  не влезает; полагание на L1 tex-cache для перекрытия warp-соседних
  Sobel-выборок корректно (соседние потоки читают соседние столбцы,
  L1-линия держит 32 пикселя = 128 байт).
- **Shared memory budget** 8 KB → 6 WG/SMX → 75 % occupancy — верно
  (blocks/thread-лимит бьёт раньше, чем warps-резидентный).
- **Barrier count** 10/WG vs ~80/WG в прежней sub-tile версии — правильный
  трейдофф.

### P3. Warp-synchronous редукция

Последние 5 шагов редукции (`s=16,8,4,2,1`) происходят внутри одного warp,
где на Kepler потоки идут lockstep'ом. В CUDA это классический
`volatile shared`-unroll без `__syncthreads()`. В GLSL/SPIR-V **нельзя
безопасно опустить** `barrier()` без `subgroup`-extension'ов (Vulkan 1.0 на
Tegra K1 их не гарантирует). **Оставить как есть** — спекулятивная
оптимизация не стоит риска недетерминизма.

### Наблюдение: глобальные атомики на финальном merge

```glsl
if (tid < 128u) {
    uint sum = 0u;
    for (uint w = 0u; w < WARPS_PER_WG; w++) sum += sHist[w][tid];
    atomicAdd(stats.lumaHist[tid], sum);
}
```

256 WG × 128 atomicAdd = **32K глобальных атомиков на кадр**, по 128
контендеров на каждый бин через L2. На Kepler глобальные атомики неплохи
(~50–100 тактов), но это верхний предел того, во что стоит упираться.

Двухстадийная редукция (писать 256 частичных гистограмм в
`partialHist[256][128]`, вторым диспатчем сводить) экономит contention ценой
одного kernel-launch'а + 128 KB трафика. **Скорее всего не выигрыш** на этом
железе — не рекомендую трогать без профилирования.

### Мелочи / проверить

- `stats.sharpness[patchIdx] = sReduce[0].w` — сохраняется **сумма**
  Tenengrad-квадратов (не нормированная на `patchW*patchH`), тогда как
  `rgbMean` нормируется. Это **намеренно** (так ожидают многие AF-consumer'ы),
  но стоит однажды свериться с тем, что ждёт наш 3A.
- `uint bin = uint(clamp(centerL * 128.0, 0.0, 127.0))` — luma-коэффициенты
  BT.601 full-range дают `[0,1]`, clamp к 127 через float корректен.

### Не трогать без профилирования

9 `dot(rgb, LUMA_COEF)` на выходной пиксель (центр + 8 соседей) — каждый
поток делает свежий dot для Sobel-соседа, хотя та же luma пересчитывается 9
раз между соседними тредами. Но L1 tex-cache делает `texelFetch` дешёвым, а
сам `dot3` — 4 такта. В сумме ~35 тактов/пиксель на luma — **в разы дешевле**
RGB-fetch'а патча. Sub-tile подход был отменён ранее по причине 80+
барьеров/WG — **решение правильное**.

---

## Порядок работ, если будете применять

1. **`DemosaicCompute` → `readPixel` в `float` с предвычисленным
   `denomRecip255`.** Минимальный diff (поле в `IspParams`, одна функция в
   шейдере), без архитектурных последствий.
2. **`DemosaicCompute` → shared-тайл для Bayer.** 12×12 для WG 8×8
   (простой путь) или 20×20 для WG 16×16 (сразу с occupancy-bump'ом). Это
   та самая "большая" оптимизация с ощутимыми миллисекундами.
3. **`DemosaicCompute` → 256-элементный LUT для sRGB-гаммы** (или sRGB
   ImageView на scratch — архитектурно чище, больше работы).
4. Остальное — `Blit` и `RgbaToNv12` косметика, `StatsCompute` не трогать.
