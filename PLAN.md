# План разработки `SPMeshHealer`

> **Цель:** Создать универсальный высокопроизводительный инструмент ремонта мешей и 3D-сканов на базе **CGAL**, который одинаково хорошо работает как **модуль/мост для `SPSculpt`** и как **однокнопочный плагин для `ZBrush` (2021–2026+)**, без жёстко прописанных путей и с поддержкой всех форматов.

---

## 1. Структура проекта `C:\Users\user\Desktop\cpp\SPMeshHealer`

```text
SPMeshHealer/
├── PLAN.md                         # Этот план
├── CMakeLists.txt                  # Сборка SPMeshHealer.exe (MSYS2 UCRT64 / MSVC + vcpkg)
├── build_ucrt64.bat                # Скрипт быстрой сборки через C:\msys64\ucrt64
├── src/
│   ├── main.cpp                    # CLI-парсер, автоопределение формата, конфиг, логи
│   ├── HealerCore.h / .cpp         # CGAL-ядро (ремонт топологии, заполнение дыр, PolyGroup)
│   ├── BinaryIO.h / .cpp           # Сырой бинарный формат (.bin) для SPSculpt
│   ├── ObjIO.h / .cpp              # Чтение/запись OBJ с сохранением индексов и PolyGroup
│   ├── GozIO.h / .cpp              # Чтение/запись GoZ binary (вершины, грани, PolyGroup, Mask, UV)
│   └── StlIO.h / .cpp              # Чтение/запись STL (fallback для сырых 3D-сканов)
├── zbrush_plugin/
│   ├── SPMeshHealer.txt            # 1-кнопочный ZScript-плагин (ZBrush 2021–2026+)
│   ├── SPMeshHealer.py             # Опциональный Python-плагин (ZBrush 2026+)
│   └── SPMeshHealerData/
│       ├── SPMeshHealer.exe        # Собранный бинарник
│       ├── ZFileUtils64.dll        # Для запуска .exe и ожидания завершения из ZScript
│       ├── config.txt              # Настройки по умолчанию
│       └── *.dll                   # Runtime-зависимости CGAL/GMP/MPFR (если динамическая линковка)
└── dist/
    └── SPMeshHealer.exe            # Готовый релизный бинарник (для копирования в SPSculpt и ZBrush)
```

---

## 2. Ядро `HealerCore`

Пайплайн обработки на базе CGAL Polygon Mesh Processing:

1. **Очистка Polygon Soup (строго последовательно):**
   - `PMP::repair_polygon_soup(points, polygons)`
   - `PMP::orient_polygon_soup(points, polygons)`
   - `PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh)`
   *(Если вход уже топологически связный, как из `GoZ`/`OBJ`/`--bin`, можно опционально пропускать soup-пересборку или включать её флагом `--repair-soup` для сохранения исходной индексации/PolyGroup там, где меш не битый.)*
2. **Триангуляция (если на входе есть квады/н-гоны):**
   - `PMP::triangulate_faces(mesh)` (с сохранением карты соответствия исходных граней и их `PolyGroup`).
3. **Предварительная очистка вырожденной геометрии:**
   - `PMP::remove_degenerate_edges(mesh)`
   - `PMP::remove_almost_degenerate_faces(mesh)`
   - `PMP::remove_degenerate_faces(mesh)`
   - `PMP::remove_isolated_vertices(mesh)`
4. **Опция `Ignore Largest Hole` (`--hull 1`):**
   - `PMP::extract_boundary_cycles(mesh, ...)`
   - Параллельный или быстрый расчёт 3D-bbox/периметра каждой граничной петли; самая большая петля помечается как внешняя оболочка и **не закрывается** при `hull == 1`.
5. **Умное заполнение отверстий:**
   - `PMP::triangulate_refine_and_fair_hole(mesh, h, ...)` для каждой выбранной петли.
   - Все вновь созданные грани-заплатки помечаются новым `PolyGroup ID` (для `.GoZ` / `.obj` / `--bin`).
6. **Удаление мелких фрагментов (опционально `--remove-fragments`):**
   - `PMP::connected_components(mesh, ...)` → удаление островков меньше `max(minFaces, totalFaces * minFraction)` (с сохранением главной компоненты).
7. **Проверка ориентации и финальная очистка:**
   - `if (!PMP::is_outward_oriented(mesh)) PMP::reverse_face_orientations(mesh);` (только для замкнутых мешей или при `--fix-orientation`).
   - Повторный проход `remove_degenerate_*` и `remove_isolated_vertices`.

---

## 3. Универсальные режимы ввода/вывода и пути (без хардкода)

### 3.1. Никаких жёстко прописанных путей
- `SPMeshHealer.exe` определяет свою директорию через `GetModuleFileNameW` (Windows) / `/proc/self/exe` (Linux).
- `config.txt` ищется в следующем порядке:
  1. Путь из `--config <path>`
  2. Рядом с `SPMeshHealer.exe`
- Временные файлы (при запуске без `--in` / `--out`) никогда не пишутся в `Program Files`: используется `%TEMP%\SPMeshHealer\`.

### 3.2. Поддержка всех форматов (автоопределение по расширению или флагу)
| Формат | Флаг / Расширение | Назначение | Особенности |
| :--- | :--- | :--- | :--- |
| **Raw Binary** | `--bin` / `.spbin` | **SPSculpt** | Максимальная скорость (`verts`, `tris`, `faceGroups`, `hull`), 0 парсинга текста. |
| **GoZ Binary** | `.GoZ` | **ZBrush** (основной) | Быстрый бинарник; переносит PolyGroup, Mask, UV, Polypaint; красит заплатки в новую PolyGroup. |
| **OBJ** | `.obj` | **ZBrush / Универсальный** | Работает везде без доп. плагинов, сохраняет общие вершины и PolyGroup (`g ...`). |
| **STL** | `.stl` | **3D-сканы / Fallback** | Поддержка бинарного и ASCII STL с полным `repair_polygon_soup`. |

### 3.3. CLI-интерфейс
```text
SPMeshHealer.exe --in <file> --out <file> [options]
  --bin                 Принудительно сырой бинарный формат (для SPSculpt)
  --hull <0|1>          1 = не закрывать самое большое отверстие (по умолчанию 0)
  --repair-soup <0|1>   1 = принудительный repair_polygon_soup (авто для STL)
  --remove-fragments    Удалять мелкие оторванные островки
  --min-faces <N>       Мин. число граней островка (по умолчанию 50)
  --min-frac <F>        Мин. доля граней островка (по умолчанию 0.01)
  --config <path>       Путь к config.txt
```

---

## 4. Однокнопочный плагин для ZBrush (`zbrush_plugin/`)

### 4.1. ZScript-версия (`SPMeshHealer.txt` — ZBrush 2021–2026+)
- **Интерфейс в `ZPlugin → SPMeshHealer`:**
  - **1 главная кнопка: `Heal Mesh`** (выполняет весь цикл за 1 клик).
  - Компактные опции под кнопкой (сохраняются в конфиг):
    - Переключатель **`Ignore Largest Hole (Hull)`** (on/off).
    - Переключатель формата: **`GoZ`** (по умолчанию, быстрее + PolyGroup) / **`OBJ`** / **`STL`**.
    - Галочка **`Remove Small Fragments`**.
- **Цикл работы одной кнопки `Heal Mesh`:**
  1. Проверить наличие `SubDiv > 1` (предупредить/перейти на нижний уровень или работать на текущем).
  2. Сформировать пути во временной папке `%TEMP%` (через `ZFileUtils64.dll` или `FileNameResolvePath`) — никаких `Program Files`!
  3. Экспортировать текущий SubTool (`[FileNameSetNext, ...][IPress, Tool:Export]`).
  4. Синхронно запустить `SPMeshHealer.exe` через `ZFileUtils64.dll` (`LaunchAppWithFile`) с аргументами `--in ... --out ... --hull ...`.
  5. Импортировать результат обратно в SubTool (`[FileNameSetNext, ...][IPress, Tool:Import]`).
  6. Удалить временные файлы.

### 4.2. Python-версия (`SPMeshHealer.py` — ZBrush 2026+)
- Аналогичная 1-кнопочная панель через `zbrush` API + `subprocess.run(...)` и `tempfile.gettempdir()`.

---

## 5. Интеграция с `SPSculpt` (мост в `MeshMend`)

В `c:\Users\user\Desktop\cpp\sculptsp-native`:

1. **`MeshMend::closeHolesCGAL(Mesh&, SelectionManager&, const CloseHolesOptions&)`:**
   - Ищет `SPMeshHealer.exe` рядом с `SPSculpt.exe` (в `dist/` или `build/`).
   - Если `SPMeshHealer.exe` **не найден** — автоматически откатывается на нативный `MeshMend::closeHolesSmart`.
   - Если найден — сериализует триангулированный меш (или окрестность дыр) в `%TEMP%\sp_heal_in.spbin`, вызывает `SPMeshHealer.exe --bin ...` (скрытое окно через `CreateProcessW` с `CREATE_NO_WINDOW`), читает `sp_heal_out.spbin` и обновляет `Mesh` + `PolyGroup` + `BVH`.
2. **UI в `GuiManager.cpp` (`Component Tools`):**
   - Переключатель движка в `Close Holes`: **CGAL (SPMeshHealer)** / **Native** (если `.exe` доступен, `CGAL` выбран по умолчанию).
   - Поддержка `Ignore Largest Hole (Hull)` и `Remove Small Fragments` прямо через `SPMeshHealer`.

---

## 6. Порядок реализации

1. **Шаг 1 — Ядро и сборка `SPMeshHealer.exe`:**
   - Создать `CMakeLists.txt` + `src/main.cpp`, `HealerCore`, `BinaryIO`, `StlIO`, `ObjIO`, `GozIO`.
   - Собрать через `C:\msys64\ucrt64` (где уже установлен CGAL) и проверить на тестовом STL/OBJ.
2. **Шаг 2 — Мост в `SPSculpt`:**
   - Добавить вызов `SPMeshHealer.exe --bin` в `MeshMend` + скопировать `SPMeshHealer.exe` (и нужные DLL, если не статическая линковка) в `sculptsp-native/_build_/dist/`.
   - Протестировать закрытие сложных дыр в `SPSculpt.exe`.
3. **Шаг 3 — 1-кнопочный плагин для ZBrush:**
   - Написать `SPMeshHealer.txt` (с `ZFileUtils64.dll`), упаковать в `zbrush_plugin/SPMeshHealerData/` и проверить в ZBrush.
