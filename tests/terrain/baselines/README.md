# Visual regression baselines

Reference screenshots for the terrain renderer live here.

Raylib on macOS needs a real window — there is no headless rendering path for
the native build — so these are captured by hand rather than by the test
harness.

## Reference frames

| File | Camera | Pose | What it shows |
|---|---|---|---|
| `chase_cam_30s.png` | Chase | replay paused at t=30 s | Terrain mesh visible after the `SIH_TERR_*` apply at t≈7 s |
| `ortho_top_30s.png` | Ortho Top | replay paused at t=30 s | Ortho path — full concentric rings rather than frustum-shaped |

## Capture procedure

Build first:

```sh
git submodule update --init --recursive
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build --config Release -j10
```

### Chase camera

```sh
./build/hawkeye --replay tests/terrain/fixtures/sih_terrain_on.ulg -w 1280 -h 720
```

1. Wait for the `home=` line in the console.
2. Chase camera is the default view (`C` cycles cameras).
3. `Space` to start playback.
4. After ~8 s of playback — past the t=7.1 s `SIH_TERR_EN` flip — press
   `Space` to pause.
5. Step forward with `Right` (+5 s) to roughly t=30 s, reading the timeline
   overlay.
6. Capture the window and save as `baselines/chase_cam_30s.png`.

### Ortho top

With the same fixture paused at t=30 s:

1. `Alt+2` switches to Top ortho.
2. `Alt+Scroll` to a span of roughly 200 m.
3. Capture and save as `baselines/ortho_top_30s.png`.

## Comparing against a baseline

```sh
python3 -c "
from PIL import Image, ImageChops
import statistics
a = Image.open('tests/terrain/baselines/chase_cam_30s.png').convert('RGB')
b = Image.open('new_capture.png').convert('RGB')
d = ImageChops.difference(a, b)
print('diff bbox:', d.getbbox())
print('mean per-channel diff: %.2f / 255'
      % statistics.mean(sum(p) / 3 for p in d.getdata()))
"
```

These are not pixel-exact comparisons — LOD selection reshapes geometry
between builds. A mean per-channel difference under roughly 30/255 means the
character of the frame is preserved; anything larger is worth looking at
directly.

## Frame rate

Independent of the screenshots, with the fixture playing past t=7.1 s:

1. `Ctrl+D` opens the debug panel and its frame-time graph.
2. Sample three readings across a 10 s window; the mean must be ≥ 30 FPS.
3. Repeat in Top ortho (`Alt+2`).
4. For WASM, serve `wasm/build/` over HTTP, load the fixture through the file
   picker, and record 10 s in the browser's performance panel.

Targets: ≥ 30 FPS for both the chase camera under WebGL2/WASM and the ortho
view with full rings.
